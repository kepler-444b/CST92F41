/*
 * app_main.c
 *
 *  Created on: 2021/3/22
 *      Author: mi
 */

#include "miio_user_api.h"
#include "mible_log.h"

#define TEST_BLE_GATEWAY        0
#define TEST_GATT_SPEC          1
#define TEST_STDIO_SERV         0//1
#define BIND_CONFIRM_TIMEOUT    (10*1000)
#define MIBEACON_ADV_TIMEOUT    (30*60*1000)
#define MIBEACON_ADV_ON         (0xFFFFFFFF)
#define MIBEACON_ADV_OFF        (0)

static void *mibeacon_bind_confirm_timer = NULL;
uint16_t mi_connect_handle = 0xFFFF;

static const uint8_t support_devinfo[] = {
    DEV_INFO_HARDWARE_VERSION,
    DEV_INFO_NEW_SN,
};

static void user_devinfo_callback(dev_info_type_t type, dev_info_t* buf)
{
    switch(type){
    case DEV_INFO_SUPPORT:
        buf->len = sizeof(support_devinfo);
        memcpy(buf->buff, support_devinfo, buf->len);
        break;
    case DEV_INFO_HARDWARE_VERSION:
        buf->len = strlen("CST92F41");
        memcpy(buf->buff, "CST92F41", buf->len);
        break;
    case DEV_INFO_NEW_SN:
        // maxlen 62
        buf->len = MIN(strlen("123456789/123456789/123456789/123456789/123456789/123456789/12"), 62);
        memcpy(buf->buff, "123456789/123456789/123456789/123456789/123456789/123456789/12", buf->len);
        break;
    default:
        buf->code = MI_ERR_NOT_FOUND;
        return;
    }
    buf->code = MI_SUCCESS;
    return;
}

static void app_dfu_callback(mible_dfu_state_t state, mible_dfu_param_t *param)
{
    switch (state)
    {
    case MIBLE_DFU_STATE_START:
        MI_LOG_INFO("fragment size is %d\n", param->start.fragment_size);
        break;
    case MIBLE_DFU_STATE_UPGRADE_STATUS:
        MI_LOG_INFO("upgrad status is %x\n", param->upgrade_status.req);
        param->upgrade_status.rsp = param->upgrade_status.req;
        break;
    case MIBLE_DFU_STATE_TRANSFER:
        MI_LOG_INFO("last fragment index is %d\n", param->trans.last_index);
        break;
    case MIBLE_DFU_STATE_VERIFY:
        MI_LOG_INFO("verify result is %d\n", param->verify.value);
        break;
    case MIBLE_DFU_STATE_SWITCH:
        MI_LOG_INFO("switch to new firmware\n");
        break;
    case MIBLE_DFU_STATE_CANCEL:
        MI_LOG_INFO("the sequence is canceled\n");
        break;
    default:
        MI_LOG_INFO("state of DFU is unknown %d\n", state);
        break;
    }
}

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch (p_event->id) {
    case SCHD_EVT_KEY_DEL_FAIL:
    case SCHD_EVT_KEY_DEL_SUCC:
        miio_system_reboot();
        break;
    case SCHD_EVT_REG_SUCCESS:
    case SCHD_EVT_KEY_FOUND:
        miio_ble_user_adv_init(0);
        miio_ble_user_adv_start(500);
        miio_ble_set_adv_timeout(MIBEACON_ADV_ON);
        break;
    case SCHD_EVT_KEY_NOT_FOUND:
        miio_ble_user_adv_init(0);
        miio_ble_user_adv_start(100);
        miio_ble_set_adv_timeout(MIBEACON_ADV_TIMEOUT);
        break;
    default:
        break;
    }
}

static void gap_event_handler(mible_gap_evt_t evt, mible_gap_evt_param_t* param)
{
    switch (evt) {
    case MIBLE_GAP_EVT_CONNECTED:
        mi_connect_handle = param->conn_handle;
        break;
    case MIBLE_GAP_EVT_DISCONNECT:
        mi_connect_handle = 0xFFFF;
        break;
    default:
        break;
    }
}

static void enqueue_new_objs(void)
{
    static float temp = 0.0f;
    static float hum = 0.0f;
    static int8_t  battery;

    temp = temp < 50.0f ? temp + 1.0f : -50.0f;
    hum = hum < 100.0f ? hum + 1.0f : 0.0f;
    battery = battery < 100 ? battery + 1 : 0;

    miio_ble_property_changed(2, 1003, property_value_new_uchar(battery), 0);
    miio_ble_property_changed(3, 1001, property_value_new_float(temp), 0);
    miio_ble_property_changed(3, 1008, property_value_new_float(hum), 1);

    miio_ble_event_occurred(2, 1001, NULL, 0);
}

#if TEST_BLE_GATEWAY
static void *gatewaytest_timer;
static void gatewaytest_handler(void *p_context)
{
    static uint32_t report_times = 0;

    if(miio_ble_get_registered_state()) {
        enqueue_new_objs();
    }

    report_times++;
    if (report_times >= 1000){
		//stop timer
		report_times = 0;
		miio_timer_stop(gatewaytest_timer);
	}
}
#endif

#if TEST_GATT_SPEC
const char *str1 = "Hi, Susan, We re throwing a surprise party for Xiaomings birthday. \
As scheduled, all the classmates in our class will gather at the Students  Centre at \
8 pm this Friday evening to celebrate Xiaoming s birthday. We would feel much honored \
if you could come and enjoy it, which is to finish at 9, lasting an hour. As the party \
starts, we will light the candles and sing Happy Birthday for him. Then the birthday \
cake will be presented and cut for all of us to enjoy, following which we ll sing songs \
and play some small games. What s more, as a surprise, we ll all prepare some secret \
birthday gifts for him, and it ll be great to see how surprised he will be seeing the \
gifts. As it s a surprise party, he will know nothing about it beforehand, and we do hope\
that you could manage to attend it. Looking forward to your coming. Yours Li Hua";

void on_property_set(property_operation_t *o)
{
    MI_LOG_INFO("[on_property_set] siid %d, piid %d\n", o->siid, o->piid);

    if (o->value == NULL)
    {
        o->code = OPERATION_ERROR_VALUE;
        MI_LOG_ERROR("value is NULL\n");
        return;
    }

    o->code = 0;
}

void on_property_get(property_operation_t *o)
{
    MI_LOG_INFO("[on_property_get] siid %d, piid %d\n", o->siid, o->piid);
    if(o->siid == 1)
        o->value = property_value_new_string(str1);
    else
        o->value = property_value_new_float(123.45f);
}

void on_action_invoke(action_operation_t *o)
{
    MI_LOG_INFO("[on_action_invoke] siid %d, aiid %d, in props %d\n", o->siid, o->aiid, o->in->size);
    o->code = 0;

    o->out->size = 3;
    o->out->arguments[0].piid = 1;
    o->out->arguments[0].value = property_value_new_uchar(80);
    o->out->arguments[1].piid = 2;
    o->out->arguments[1].value = property_value_new_float(-12.3f);
    o->out->arguments[2].piid = 3;
    o->out->arguments[2].value = property_value_new_string("abcdefg12345");
}

void gatt_prop()
{
    if(mi_connect_handle != 0xFFFF){
    //3-1021 Door state, 4-1003 Battery, 6-1018 Custom property
//    miio_gatt_properties_changed(3, 1021, property_value_new_uchar(0));
//    miio_gatt_properties_changed(4, 1003, property_value_new_uchar(100));
//    miio_gatt_properties_changed(6, 1018, property_value_new_uchar(50));
        miio_gatt_property_changed(1, 2, property_value_new_uchar(80));
        miio_gatt_property_changed(2, 4, property_value_new_float(-12.3f));
        miio_gatt_property_changed(3, 6, property_value_new_string("abcdefg1234567!"));

        properties_t newProps[5];
        newProps[0].siid = 1;newProps[0].piid = 2;
        newProps[0].value = property_value_new_uchar(80);
        newProps[1].siid = 1;newProps[1].piid = 2;
        newProps[1].value = property_value_new_uchar(100);
        newProps[2].siid = 2;newProps[2].piid = 4;
        newProps[2].value = property_value_new_float(-12.3f);
        newProps[3].siid = 2;newProps[3].piid = 4;
        newProps[3].value = property_value_new_float(23.4f);
        newProps[4].siid = 3;newProps[4].piid = 6;
        newProps[4].value = property_value_new_string(str1);
        miio_gatt_properties_changed(5, newProps);
    }
}

void gatt_event()
{
    if(mi_connect_handle != 0xFFFF){
        //5-1006 Door Ring
        miio_gatt_event_occurred(5, 1006, NULL);
        //2-1007 Lock abnormal
        arguments_t *newArgs = arguments_new();
        newArgs->size = 2;
        newArgs->arguments[0].piid = 4;
        newArgs->arguments[0].value = property_value_new_uchar(3);
        newArgs->arguments[1].piid = 6;
        newArgs->arguments[1].value = property_value_new_string("abcdefghijklmnopqrstuvwxyz1234567890!!!");
        miio_gatt_event_occurred(2, 1007, newArgs);
    }
}
#endif


#if TEST_STDIO_SERV
#include "mijia_profiles/stdio_service_server.h"
void stdio_rx_handler(uint8_t* p, uint8_t l)
{
    int errno;
    /* RX plain text (It has been decrypted) */
    MI_LOG_INFO("RX raw data\n");
    MI_LOG_HEXDUMP(p, l);

    /* TX plain text (It will be encrypted before send out.) */
    errno = stdio_tx(p, l);
    MI_ERR_CHECK(errno);
}
#endif

/* This function need callback when button press*/
void user_button_handler(uint8_t key_index)
{
	MI_LOG_DEBUG("user_button_handler key_index=%d.\n", key_index);
    if(key_index == 0){

        if(miio_ble_get_registered_state()) {
#if TEST_BLE_GATEWAY
            /* keepalive: at last 50min report props or event (can use battery level)*/
            /* period report: report Temperature or Humidity in 1~5min (depend on delta)*/
            uint32_t delay_ms = 10000;
            MI_LOG_INFO("start object adv after %d ms\n", delay_ms);
            miio_timer_start(gatewaytest_timer, delay_ms, NULL);
#else
            enqueue_new_objs();
#endif
#if TEST_GATT_SPEC
            if(mi_connect_handle != 0xFFFF){
                gatt_prop();
                gatt_event();
            }
#endif
        }else{
            MI_LOG_DEBUG("Set bind confirm bit in mibeacon.\n");
            miio_ble_user_adv_init(1);
            miio_timer_start(mibeacon_bind_confirm_timer, BIND_CONFIRM_TIMEOUT, NULL);
        }
    }else{
        miio_system_restore();
    }
}

static void mibeacon_bind_confirm_handler(void * p_context)
{
	MI_LOG_INFO("clear bind confirm bit !\n");
	miio_ble_user_adv_init(0);
	miio_timer_stop(mibeacon_bind_confirm_timer);
}

void user_app_init(void)
{
    mi_service_init();
    mible_gap_register(gap_event_handler);
    /* <!> mi_scheduler_init() must be called after ble_stack_init(). */
    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);

    miio_system_info_callback_register(user_devinfo_callback);
    miio_dfu_callback_register(app_dfu_callback);
    miio_timer_create(&mibeacon_bind_confirm_timer, mibeacon_bind_confirm_handler, MIBLE_TIMER_SINGLE_SHOT);

#if TEST_BLE_GATEWAY
    miio_timer_create(&gatewaytest_timer, gatewaytest_handler, MIBLE_TIMER_REPEATED);
#endif

#if TEST_GATT_SPEC
    miio_gatt_spec_init(on_property_set, on_property_get, on_action_invoke, 1024,16);
#endif

#if TEST_STDIO_SERV
    stdio_service_init(stdio_rx_handler);
#endif
}

void user_app_main_thread(void)
{
    mible_tasks_exec();
#if (MI_SCHD_PROCESS_IN_MAIN_LOOP==1)
    // Process mi scheduler
    mi_schd_process();
#endif
    return;
}
