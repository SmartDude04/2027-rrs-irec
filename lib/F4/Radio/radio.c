#include "radio.h"

uint8_t radio_checkin(struct radio_link *link, uint8_t destination) {
    if (!radio_link_send(link, destination, MSG_HEARTBEAT, NULL, 0)) {
        return COMM_TX_FAILED;
    }
    return COMM_OK;
}

uint8_t radio_command(struct radio_link *link, uint8_t destination, uint8_t command) {
    if (!radio_link_send(link, destination, MSG_COMMAND, &command, 1)) {
        return COMM_TX_FAILED;
    }
    return COMM_OK;
}

uint8_t radio_request_data(struct radio_link *link, uint8_t destination) {
    if (!radio_link_send(link, destination, MSG_TELEMETRY, NULL, 0)) {
        return COMM_TX_FAILED;
    }
    return COMM_OK;
}

void radio_rx_process(struct radio_link *link) {
    struct protocol_packet rx_packet = {0};
    while(radio_link_message_available(link)) { // Go through queue
        radio_link_get_message(link, &rx_packet); // Get first message off queue

        switch (rx_packet.message_type)
        {
            case MSG_HEARTBEAT:
                // Could have it send back radio stats
                radio_link_send(link, rx_packet.source, MSG_HEARTBEAT_ACK, NULL, 0);
                break;
            case MSG_HEARTBEAT_ACK:
                // TODO: Record that node is still active
                radio_flag = MSG_HEARTBEAT_ACK + rx_packet.source;
                break;
            case MSG_COMMAND:
                // TODO: Implement way to execute commands, maybe an external command queue that is called in the main loop
                // after command is executed it should then send corresponding CMD_ACK
                radio_link_send(link, rx_packet.source, MSG_COMMAND_ACK, NULL, 0);
                break;
            case MSG_COMMAND_ACK:
                command_status = rx_packet.data;
                // The data shold be of type command_ack
                // Check which one it was and save to radio_status
                break;
            case MSG_TELEMETRY:
                radio_flag = MSG_TELEMETRY;
                break;
            case MSG_TELEMETRY_ACK:
                // Regular sensor data
                // Convert rx_packet.data to corresponding packet struct depending on source
                if (rx_packet.source == NODE_AIRBRAKES) {
                    radio_flag = MSG_TELEMETRY_ACK + NODE_AIRBRAKES;
                    // Save rx_packet.data to airbrake struct
                }
                else if (rx_packet.source == NODE_PAYLOAD)
                {
                    radio_flag = MSG_TELEMETRY_ACK + NODE_PAYLOAD;
                    // Save rx_packet.data to payload struct
                }
                break;
            default:
                break;
        }
    }
}