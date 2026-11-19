#ifndef __MESSAGE_TYPES_H__
#define __MESSAGE_TYPES_H__

#define NODE_GROUND_STATION 0x01
#define NODE_AIRBRAKES      0x02
#define NODE_PAYLOAD        0x03

// MSG_xx is call and MSG_xx_ACK is response
enum message_type {
    // Call for Heartbeat
    MSG_HEARTBEAT       = 0x01,
    // Responds with heartbeat
    MSG_HEARTBEAT_ACK   = 0x02,

    // Calls for telemetry data
    MSG_TELEMETRY       = 0x03,
    // Responds with telemetry data
    MSG_TELEMETRY_ACK   = 0x04,

    MSG_COMMAND         = 0x10,
    MSG_COMMAND_ACK     = 0x11,
    MSG_COMMAND_ARM     = 0x12,

    MSG_ACK             = 0x20,
    MSG_NACK            = 0x21
};

// Can equal a message_type to tell other parts of the program what message has arrived
uint8_t radio_flag;
uint8_t command_status;

#endif