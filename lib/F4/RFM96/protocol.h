#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#define MAX_PACKET_LENGTH 128
#define MAX_DATA_LENGTH 119 //128 - non data bits


/**
 * @brief Packet format
 */
struct protocol_packet {
    uint8_t source;
    uint8_t destination;
    uint8_t message_type;
    uint16_t sequence;
    uint8_t data_length;
    uint8_t data[MAX_DATA_LENGTH];
    uint16_t crc;
};

/**
 * @brief Encodes a formatted packet into a byte array
 * @param packet formatted packet that will be encoded
 * @param buf byte array where packet will be encoded to
 * @return True if packet succesfully encoded, False if encoding failed due to malformed packet or buffer
 */
bool protocol_encode(const struct protocol_packet *packet, uint8_t *buf);


/**
 * @brief Decodes a byte array into a formatted packet
 * @param packet formatted packet where byte array will encoded to
 * @param buf byte array that will be decoded
 * @return True if packed succesfully decoded, False if decoding failed
 */
bool protocol_decode(struct protocol_packet *packet, const uint8_t *buf);

bool protocol_validate(const struct protocol_packet *packet);

uint16_t protocol_calculate_crc(const uint8_t *data, uint16_t length);

#endif