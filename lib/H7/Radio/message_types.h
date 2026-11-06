#ifndef __MESSAGE_TYPES_H__
#define __MESSAGE_TYPES_H__

#define NODE_GROUND_STATION 0x01,
#define NODE_AIRBRAKES      0x02,
#define NODE_PAYLOAD        0x03

enum message_type {
    MSG_HEARTBEAT       = 0x01,
    MSG_HEARTBEAT_ACK   = 0x02,

    MSG_TELEMETRY       = 0x03,

    MSG_COMMAND         = 0x10,
    MSG_COMMAND_ACK     = 0x11,

    MSG_ACK             = 0x20,
    MSG_NACK            = 0x21
};

enum command_type {
    CMD_REQUEST_STATUS = 0x01,
    CMD_TEST_SENSOR    = 0x02,
    CMD_TEST_FUNCTION  = 0x03,

    CMD_START_LOGGING  = 0x10,
    CMD_STOP_LOGGING   = 0x11
};

enum command_ack {
    CMD_RECEIVED,
    CMD_ACCEPTED,
    CMD_COMPLETED,
    CMD_REJECTED,
    CMD_FAILED
};

#endif