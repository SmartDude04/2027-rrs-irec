#ifndef __RADIO_H__
#define __RADIO_H__

#include "radio_link.h"
#include "protocol.h"
#include "message_types.h"

typedef enum
{
    COMM_OK = 0,
    COMM_ERROR,
    COMM_INVALID_ARGUMENT,
    COMM_TX_FAILED

} communication_status_t;

uint8_t radio_checkin(struct radio_link *link, uint8_t destination);
uint8_t radio_command(struct radio_link *link, uint8_t destination, uint8_t command);
uint8_t radio_request_data(struct radio_link *link, uint8_t destination);
uint8_t radio_rx_process(struct radio_link *link);


#endif