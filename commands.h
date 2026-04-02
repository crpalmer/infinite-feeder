#ifndef __COMMANDS_H__
#define __COMMANDS_H__

typedef enum {
    CMD_PING, CMD_PONG,
    CMD_GET_CONFIG, CMD_CONFIG_BLOB,
    CMD_NEW_CONFIG_AVAILABLE,
    CMD_GET_STATUS, CMD_STATUS,
} cmd_t;

#endif
