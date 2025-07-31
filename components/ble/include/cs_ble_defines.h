#ifndef _CS_BLE_DEFINES_H
#define _CS_BLE_DEFINES_H

#include <stdint.h>

#define BD_ADDR_LEN     6

typedef struct bd_addr {
    uint8_t addr[6];
} bd_addr_t;

extern struct bd_addr co_default_bdaddr;

#endif
