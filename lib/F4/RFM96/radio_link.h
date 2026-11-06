#ifndef  __RADIO_LINK_H__
#define __RADIO_LINK_H__

#include <stdint.h>
#include <stdbool.h>
#include "lora_sx1276.h"
#include "protocol.h"

#define RADIO_LINK_RX_QUEUE_SIZE 4

enum radio_status {
    RADIO_OK,
    RADIO_BUSY,
    RADIO_TIMEOUT,
    RADIO_ERROR,
    RADIO_CRC_ERROR,
    RADIO_NO_PACKET
};

/**
 * @brief Recieved radio transmission queue
 */
struct radio_link_queue {
    struct protocol_packet packets[RADIO_LINK_RX_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
};

/**
 * @brief Radio statistics
 */
struct radio_link_stats {
    uint16_t tx_packets;
    uint16_t rx_packets;
    uint16_t rx_crc_errors;
    uint16_t rx_sequence_gaps;

    int16_t last_rssi_dbm;
    int16_t last_snr_db;

    uint32_t last_rx_time_ms;
};

/**
 * @brief Radio connection struct
 * @param lora - lora instance
 * @param node_ide - ID number of node
 * @param sequence - increasing sequence of attempted transmitted packets
 */
struct radio_link {
    lora_sx1276 *lora;
    uint8_t node_id;
    uint16_t sequence;
    struct radio_link_queue rx_queue;
    struct radio_link_stats stats;
};

bool radio_link_init(lora_sx1276 *lora, struct radio_link *link, uint8_t node_id);

bool radio_link_send(
    struct radio_link *link,
    uint8_t destination,
    uint8_t message_type,
    const uint8_t *data,
    uint16_t data_length
);

void radio_link_receive(struct radio_link *link);

bool radio_link_message_available(struct radio_link *link);

bool radio_link_get_message(struct radio_link *link, struct protocol_packet *packet);

#endif