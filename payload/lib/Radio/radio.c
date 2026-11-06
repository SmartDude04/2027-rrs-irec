#include "radio.h"

void radio_config(lora_sx1276 *lora) {
    lora_init_ex(lora, &hspi1, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_3, 
                433000000, 7, LORA_BANDWIDTH_125_KHZ, 10, LORA_PA_OUTPUT_PA_BOOST);
}

void radio_test(lora_sx1276 *lora) {
    uint8_t buf[32];
    uint8_t res;
    uint8_t len = lora_receive_packet_blocking(lora, buf, sizeof(buf), 10000, &res);
    if (res != LORA_OK) {
        SerialWrite("LORA Failed on receive: %u\n", res);
    }
    buf[len] = 0;
    struct payload rx;
    rx.device_ID = buf[0];
    rx.data = buf[8];
    uint16_t rssi = lora_packet_rssi(lora);
    int8_t snr = lora_packet_snr(lora);
    uint32_t tick = HAL_GetTick();
    SerialWrite("Timeup: %u\tData: %u\tRSSI: %u\tSNR: %u\n", tick, rx, rssi, snr);

}