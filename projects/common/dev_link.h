#ifndef __DEV_LINK_H
#define __DEV_LINK_H

#include "cs_ble_gap.h"

typedef int (*autolink_handler)(void);

void set_autolink_handler(autolink_handler handler);
void autolink_daemon_start(void);
void autolink_daemon_stop(void);
void autolink_init(void);

#endif
