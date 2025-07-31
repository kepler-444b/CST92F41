
#include <stdint.h>
#include <string.h>
#include "cs_common_utils.h"

const char *bt_hex(const void *buf, uint16_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    static char str[129];
    const uint8_t *b = buf;
    int i;

    len = MIN(len, (sizeof(str) - 1) / 2);

    for (i = 0; i < len; i++) {
        str[i * 2] = hex[b[i] >> 4];
        str[i * 2 + 1] = hex[b[i] & 0xf];
    }

    str[i * 2] = '\0';

    return str;
}

bool str_is_valid_mac(const char *str)
{
    if (strlen(str) != 12) {
        return false;
    }

    for (int i = 0; i < strlen(str); i++) {
        char x = str[i];
        if ((x >= '0') && (x <= '9')) {
            continue;
        }
        if ((x >= 'a') && (x <= 'f')) {
            continue;
        }
        if ((x >= 'A') && (x <= 'F')) {
            continue;
        }

        return false;
    }

    return true;
}

