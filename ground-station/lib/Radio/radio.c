#include "radio.h"

uint8_t count = 0;

void radio_config(lora_sx1276 *lora) {
    lora_init_ex(lora, &hspi1, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_3, 
                433000000, 7, LORA_BANDWIDTH_125_KHZ, 10, LORA_PA_OUTPUT_PA_BOOST);
}

void radio_test(lora_sx1276 *lora) {
    lora_send_packet(lora, count, sizeof(count));
    count++;
    SerialWrite("LORA Send Packet");
}