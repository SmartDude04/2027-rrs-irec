#include "radio_link.h"
#include <string.h>

bool radio_link_init(lora_sx1276 *lora, struct radio_link *link, uint8_t node_id) {
    uint8_t ret = lora_init_ex(lora, lora->spi, lora->nss_port, lora->nss_pin, lora->rst_port, lora->rst_pin,
                    lora->frequency, lora->sf, lora->bw, lora->tx_power, LORA_PA_OUTPUT_PA_BOOST);
    if (ret != LORA_OK) {
        return ret;
    }

    link->lora = lora;
    link->node_id = node_id;
    link->sequence = 0;

    link->rx_queue.count = 0;
    link->rx_queue.head = 0;
    link->rx_queue.tail = 0;

    lora_mode_receive_continuous(lora);

    return ret;
}

uint8_t radio_link_send(struct radio_link *link, uint8_t destination, uint8_t message_type, 
                    const uint8_t *data, uint16_t data_length) {
    if (link == NULL || link->lora == NULL)
        return RADIO_ERROR;

    // 1. Create a generic packet
    struct protocol_packet packet = {0};

    packet.source = link->node_id;
    packet.destination = destination;
    packet.message_type = message_type;
    packet.sequence = link->sequence++;
    packet.data_length = data_length;

    // 2. Copy the application data into the packet
    if (data_length > 0)
    {
        memcpy(packet.data, data, data_length);
    }
    // 3. Convert the packet into a byte array
    uint8_t tx_buffer[data_length + 8];
    protocol_encode(&packet, tx_buffer);

    // 4. Give the encoded bytes to the radio
    if(lora_is_transmitting(link->lora)) {
        return RADIO_BUSY;
    }
    uint8_t ret = lora_send_packet_blocking(link->lora, tx_buffer, data_length + 8, 200);
    if (ret != LORA_OK) {
        return ret;
    }
    lora_mode_receive_continuous(link->lora);
    link->stats.tx_packets++;
    return RADIO_OK;
}

uint8_t radio_link_receive(struct radio_link *link) {
    //lora_mode_receive_continuous(link->lora);
    if (!lora_is_packet_available(link->lora)){
        return RADIO_OK;
    }
    uint8_t buf[LORA_MAX_PACKET_SIZE];
    uint8_t err;
    uint8_t len = lora_receive_packet(link->lora, buf, LORA_MAX_PACKET_SIZE, &err);
    if (err != LORA_OK) {
        return RADIO_ERROR;
    }

    struct protocol_packet rx_packet = {0};

    if (!protocol_decode(&rx_packet, buf)) {
        return RADIO_PACKET_ERROR;
    }

    if (rx_packet.destination != link->node_id) {
        return RADIO_OK;
    }

    // We could check if the queue is full before adding, but I would rather just lose transmissions then lose new transmissions

    // Add packet to queue and increment queue
    if (link->rx_queue.count >= RADIO_LINK_RX_QUEUE_SIZE) {
        // Drop the oldest packet
        link->rx_queue.tail = (link->rx_queue.tail + 1) % RADIO_LINK_RX_QUEUE_SIZE;

    }
    else {
        link->rx_queue.count++;
    }
    // Store the newest packet
    link->rx_queue.packets[link->rx_queue.head] = rx_packet;
    link->rx_queue.head = (link->rx_queue.head + 1) % RADIO_LINK_RX_QUEUE_SIZE;

    // Update stats
    link->stats.rx_packets++;
    link->stats.last_rx_time_ms = HAL_GetTick();
    link->stats.last_snr_db = lora_packet_snr(link->lora);
    link->stats.last_rssi_dbm = lora_packet_rssi(link->lora);
    return RADIO_OK;
}


bool radio_link_message_available(struct radio_link *link) {
    if (link->rx_queue.count > 0) {
        return true;
    }
    return 0;
}

bool radio_link_get_message(struct radio_link *link, struct protocol_packet *packet) {
    if (link->rx_queue.count == 0)
        return false;

    // Retreive oldest packet
    *packet = link->rx_queue.packets[link->rx_queue.tail];
    
    // Advance the queue
    link->rx_queue.tail = (link->rx_queue.tail + 1) % RADIO_LINK_RX_QUEUE_SIZE;
    link->rx_queue.count--;

    return true;
}