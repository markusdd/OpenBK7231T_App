// drv_dreo.c - Dreo Heater MCU driver for OpenBeken
// Port of https://github.com/lygris/dreo_heater (ESPHome component)
// Tiny differences from TuyaMCU: header + seq byte + checksum formula

#include "../new_common.h"
#include "drv_public.h"
#include "drv_uart.h"
#include "drv_tuyaMCU.h"   // reuse DP parser and helpers

#define LOG_FEATURE_TUYAMCU LOG_FEATURE_DREO   // reuse the same log tag or change it

static uint8_t g_dreoSeq = 0;           // rolling sequence counter
static bool g_dreoInitialised = false;

// ------------------------------------------------------------------
// Packet detection (replaces UART_TryToGetNextTuyaPacket)
// ------------------------------------------------------------------
int Dreo_TryToGetNextPacket(byte* out, int maxSize) {
    int cs = UART_GetDataSize();
    if (cs < 9) return 0;               // minimum header + checksum

    // skip garbage
    while (cs > 0) {
        if (UART_GetByte(0) == 0x55 && UART_GetByte(1) == 0xAA) break;
        UART_ConsumeBytes(1);
        cs--;
    }
    if (cs < 9) return 0;

    // header: 55 AA 00 seq cmd 00 00 len_lo
    uint16_t payloadLen = UART_GetByte(7);          // len is only 1 byte in practice
    uint16_t packetLen = 8 + payloadLen + 1;        // 8 header + payload + checksum

    if (cs < packetLen) return 0;                   // not full frame yet

    // copy whole packet
    if (packetLen > maxSize) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DREO, "Dreo packet too big (%d > %d)\n", packetLen, maxSize);
        UART_ConsumeBytes(packetLen);
        return 0;
    }
    for (int i = 0; i < packetLen; i++) out[i] = UART_GetByte(i);

    UART_ConsumeBytes(packetLen);
    return packetLen;
}

// ------------------------------------------------------------------
// Checksum (exactly as in ESPHome)
// ------------------------------------------------------------------
static uint8_t Dreo_CalcChecksum(const byte* data, int len) {
    uint32_t sum = 0;
    for (int i = 2; i < len - 1; i++) sum += data[i];   // from ver byte to end of payload
    return (uint8_t)((sum - 1) & 0xFF);
}

// ------------------------------------------------------------------
// Process incoming frame (reuses TuyaMCU DP parser)
// ------------------------------------------------------------------
static void Dreo_ProcessPacket(const byte* data, int len) {
    if (len < 9 || data[0] != 0x55 || data[1] != 0xAA) return;

    uint8_t cmd = data[4];
    uint16_t payloadLen = data[7];
    const byte* payload = data + 8;

    // verify checksum
    if (Dreo_CalcChecksum(data, len) != data[len - 1]) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DREO, "Bad Dreo checksum\n");
        return;
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_DREO, "Dreo cmd=0x%02X seq=0x%02X payloadLen=%d\n",
              cmd, data[3], payloadLen);

    switch (cmd) {
        case 0x00:                  // heartbeat reply
            g_dreoInitialised = true;
            break;

        case 0x07:                  // status report (most common)
        case 0x08:
            TuyaMCU_ParseStateMessage(payload, payloadLen);   // reuses full Tuya DP parser
            break;

        // add more cmd handlers if needed (e.g. 0x06 ack, etc.)
    }
}

// ------------------------------------------------------------------
// Send a raw Dreo frame (cmd + payload)
// ------------------------------------------------------------------
void Dreo_SendRaw(uint8_t cmd, const uint8_t* payload, size_t payloadLen) {
    uint8_t pkt[128];
    size_t totalLen = 8 + payloadLen + 1;

    if (totalLen > sizeof(pkt)) return;

    pkt[0] = 0x55;
    pkt[1] = 0xAA;
    pkt[2] = 0x00;          // fixed "version"
    pkt[3] = g_dreoSeq++;   // rolling seq
    pkt[4] = cmd;
    pkt[5] = 0x00;
    pkt[6] = 0x00;
    pkt[7] = (uint8_t)payloadLen;

    uint32_t sum = pkt[3] + pkt[4] + pkt[7];
    if (payload && payloadLen) {
        memcpy(pkt + 8, payload, payloadLen);
        for (size_t i = 0; i < payloadLen; i++) sum += payload[i];
    }

    pkt[totalLen - 1] = (uint8_t)((sum - 1) & 0xFF);

    UART_SendArray(pkt, totalLen);
}

// ------------------------------------------------------------------
// Send a single DP (exactly like TuyaMCU_SendState but via Dreo frame)
// ------------------------------------------------------------------
void Dreo_SendDP(uint8_t dpId, uint8_t type, const void* value, int dataLen) {
    uint8_t buf[64];
    size_t idx = 0;

    buf[idx++] = dpId;
    buf[idx++] = 0x01;                  // protocol byte (same as ESPHome)
    buf[idx++] = type;

    // len (2 bytes, big-endian)
    buf[idx++] = (dataLen >> 8) & 0xFF;
    buf[idx++] = dataLen & 0xFF;

    memcpy(buf + idx, value, dataLen);
    idx += dataLen;

    Dreo_SendRaw(0x06, buf, idx);       // cmd 0x06 = set DP
}

// ------------------------------------------------------------------
// Convenience wrappers (power, mode, target temp, etc.)
// ------------------------------------------------------------------
void Dreo_SetPower(bool on) {
    uint8_t v = on ? 1 : 0;
    Dreo_SendDP(1, DP_TYPE_BOOL, &v, 1);   // DP01 = power
}

void Dreo_SetMode(uint8_t mode) {          // 1=manual/Hx, 2=eco, 3=fan-only (from ESPHome)
    Dreo_SendDP(2, DP_TYPE_ENUM, &mode, 1);
}

void Dreo_SetTargetTemp(float celsius) {
    uint8_t v = (uint8_t)celsius;          // byte value, same scaling as ESPHome
    Dreo_SendDP(4, DP_TYPE_VALUE, &v, 1);
}

// ... add more as needed (sound=DP06, display=DP08, childlock=DP16, etc.)

// ------------------------------------------------------------------
// Driver init / main loop
// ------------------------------------------------------------------
void Dreo_Init() {
    addLogAdv(LOG_INFO, LOG_FEATURE_DREO, "Dreo MCU driver started\n");

    // same init sequence as ESPHome
    Dreo_SendRaw(0x00, NULL, 0);
    DelayMs(50);
    uint8_t init1[] = {0x02, 0x05, 0x00};
    Dreo_SendRaw(0x03, init1, sizeof(init1));
    DelayMs(50);
    Dreo_SendRaw(0x02, NULL, 0);

    g_dreoInitialised = true;
}

void Dreo_OnEverySecond() {
    if (!g_dreoInitialised) return;

    static uint32_t lastHb = 0;
    if (TimeSinceStartup() - lastHb > 10) {     // 10 s heartbeat
        Dreo_SendRaw(0x00, NULL, 0);
        lastHb = TimeSinceStartup();
    }
}

// ------------------------------------------------------------------
// UART RX handler (called from main UART loop)
// ------------------------------------------------------------------
void Dreo_OnUartRx() {
    byte buf[256];
    int len;
    while ((len = Dreo_TryToGetNextPacket(buf, sizeof(buf))) > 0) {
        Dreo_ProcessPacket(buf, len);
    }
}

// Register the driver (add this to drv_main.c or use the online builder)
void DRV_DREO_Init() {
    DRV_RegisterDriver("Dreo", Dreo_OnUartRx, Dreo_OnEverySecond);
    // optional: add command handlers, e.g. "dreoPower 1", "dreoTemp 25" etc.
}
