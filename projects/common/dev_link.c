
#include "sdk_config.h"
#include "app_scan.h"
#include "cs_ble_error.h"
#include "dev_link.h"
#include "evt_timer.h"
#include "user_data.h"

#ifdef AT_ENABLE

static evt_timer_t autolink_timer;
static autolink_handler auto_link_handler = NULL;

void set_autolink_handler(autolink_handler handler)
{
    auto_link_handler = handler;
}

static void autolink_timer_handler(evt_timer_t *timer, void *param)
{
    CS_LOG_DEBUG("%s: auto_link_handler %p\r\n", __func__, auto_link_handler );
    if ((auto_link_handler != NULL) &&
            auto_link_handler()) {
        evt_timer_set(&autolink_timer, 1000, EVT_TIMER_ONE_SHOT, autolink_timer_handler, NULL);
    } else {
        evt_timer_del(&autolink_timer);
    }
}

void autolink_daemon_start(void)
{
    at_ble_mode_t ble_mode = user_data_get_ble_mode();
    if (ble_mode != AT_BLE_MODE_SLAVE_ONLY ) {
        evt_timer_set(&autolink_timer, 1000, EVT_TIMER_ONE_SHOT, autolink_timer_handler, NULL);
    }
}

void autolink_daemon_stop(void)
{
    evt_timer_del(&autolink_timer);
}

void autolink_init(void)
{
    autolink_daemon_start();
}

#endif
