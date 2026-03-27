// drv_dreo.c - Dreo Heater MCU driver for OpenBeken
// Port of https://github.com/lygris/dreo_heater (ESPHome component)

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "drv_public.h"
#include "drv_uart.h"
#include "drv_tuyaMCU.h"

// DP types (copied from drv_tuyaMCU.c - they are not in the header)
#define DP_TYPE_RAW          0x00
#define DP_TYPE_BOOL         0x01
#define DP_TYPE_VALUE        0x02
#define DP_TYPE_STRING       0x03
#define DP_TYPE_ENUM         0x04
#define DP_TYPE_BITMAP       0x05

// Internal TuyaMCU parser (not exported in header)
extern void TuyaMCU_ParseStateMessage(const byte* data, int len);

#define LOG_FEATURE LOG_FEATURE_TUYAMCU

static uint8_t g_dreoSeq = 0;
static bool g_dreoInitialised = false;

// ------------------------------------------------------------------
// Packet detection
// ------------------------------------------------------------------
int Dreo_TryToGetNextPacket(byte* out, int maxSize) {
    int cs = UART_GetDataSize();
    if (cs < 9) return 0;

    // skip garbage
    while (cs > 0) {
        if (UART_GetByte(0) == 0x55 && UART_GetByte(1) == 0xAA) break;
        UART_ConsumeBytes(1);
        cs--;
    }
    if (cs < 9) return 0;

    uint16_t payloadLen = UART_GetByte(7);
    uint16_t packetLen = 8 + payloadLen + 1;

    if (cs < packetLen) return 0;

    if (packetLen > maxSize) {
        addLogAdv(LOG_INFO, LOG_FEATURE, "Dreo packet too big (%d > %d)\n", packetLen, maxSize);
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
    for (int i = 2; i < len - 1; i++) sum += data[i];
    return (uint8_t)((sum - 1) & 0xFF);
}

// ------------------------------------------------------------------
// Process incoming frame
// ------------------------------------------------------------------
static void Dreo_ProcessPacket(const byte* data, int len) {
    if (len < 9 || data[0] != 0x55 || data[1] != 0xAA) return;

    uint8_t cmd = data[4];
    uint16_t payloadLen = data[7];
    const byte* payload = data + 8;

    if (Dreo_CalcChecksum(data, len) != data[len - 1]) {
        addLogAdv(LOG_INFO, LOG_FEATURE, "Bad Dreo checksum\n");
        return;
    }

    addLogAdv(LOG_INFO, LOG_FEATURE, "Dreo cmd=0x%02X seq=0x%02X payloadLen=%d\n",
              cmd, data[3], payloadLen);

    switch (cmd) {
        case 0x00:                  // heartbeat reply
            g_dreoInitialised = true;
            break;

        case 0x07:                  // status report
        case 0x08:
            TuyaMCU_ParseStateMessage(payload, payloadLen);
            break;
    }
}

// ------------------------------------------------------------------
// Send raw frame (using UART_SendByte loop - the only available function)
// ------------------------------------------------------------------
void Dreo_SendRaw(uint8_t cmd, const uint8_t* payload, size_t payloadLen) {
    uint8_t pkt[128];
    size_t totalLen = 8 + payloadLen + 1;

    if (totalLen > sizeof(pkt)) return;

    pkt[0] = 0x55;
    pkt[1] = 0xAA;
    pkt[2] = 0x00;
    pkt[3] = g_dreoSeq++;
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

    // send byte-by-byte
    for (size_t i = 0; i < totalLen; i++) {
        UART_SendByte(pkt[i]);
    }
}

// ------------------------------------------------------------------
// Send single DP (exactly like TuyaMCU)
// ------------------------------------------------------------------
void Dreo_SendDP(uint8_t dpId, uint8_t type, const void* value, int dataLen) {
    uint8_t buf[64];
    size_t idx = 0;

    buf[idx++] = dpId;
    buf[idx++] = 0x01;                  // protocol byte
    buf[idx++] = type;

    buf[idx++] = (dataLen >> 8) & 0xFF;
    buf[idx++] = dataLen & 0xFF;

    memcpy(buf + idx, value, dataLen);
    idx += dataLen;

    Dreo_SendRaw(0x06, buf, idx);
}

// ------------------------------------------------------------------
// Convenience functions (feel free to add more DPs)
// ------------------------------------------------------------------
void Dreo_SetPower(bool on) {
    uint8_t v = on ? 1 : 0;
    Dreo_SendDP(1, DP_TYPE_BOOL, &v, 1);
}

void Dreo_SetMode(uint8_t mode) {          // 1=manual, 2=eco, 3=fan-only
    Dreo_SendDP(2, DP_TYPE_ENUM, &mode, 1);
}

void Dreo_SetTargetTemp(uint8_t celsius) {
    Dreo_SendDP(4, DP_TYPE_VALUE, &celsius, 1);
}

// ------------------------------------------------------------------
// Driver init
// ------------------------------------------------------------------
void Dreo_Init() {
    addLogAdv(LOG_INFO, LOG_FEATURE, "Dreo MCU driver started\n");

    // Make sure UART is initialised for the heater (115200 is standard)
    UART_InitUART(115200, 0, false);
    UART_InitReceiveRingBuffer(1024);

    // same init sequence as ESPHome
    Dreo_SendRaw(0x00, NULL, 0);
    delay_ms(50);
    uint8_t init1[] = {0x02, 0x05, 0x00};
    Dreo_SendRaw(0x03, init1, sizeof(init1));
    delay_ms(50);
    Dreo_SendRaw(0x02, NULL, 0);

    g_dreoInitialised = true;
}

void Dreo_OnEverySecond() {
    if (!g_dreoInitialised) return;

    static uint32_t lastHb = 0;
    if (g_secondsElapsed - lastHb > 10) {
        Dreo_SendRaw(0x00, NULL, 0);
        lastHb = g_secondsElapsed;
    }
}

void Dreo_OnUartRx() {
    byte buf[256];
    int len;
    while ((len = Dreo_TryToGetNextPacket(buf, sizeof(buf))) > 0) {
        Dreo_ProcessPacket(buf, len);
    }
}

// ------------------------------------------------------------------
// Entry point (called by the framework when you do "startDriver Dreo")
// ------------------------------------------------------------------
void DRV_DREO_Init() {
    Dreo_Init();
}