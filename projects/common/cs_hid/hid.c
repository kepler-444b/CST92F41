/**
 ****************************************************************************************
 *
 * @file hogpd.c
 *
 * @brief HID Over GATT Profile HID Device Role Implementation.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HOGPD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

#include "sdk_config.h"
#include "cs_ble.h"
#include "cs_ble_gatt.h"
#include "cs_common_utils.h"
#include "cs_log.h"
#include "hid.h"
#include "evt.h"

#if (BLE_HID_DEVICE)

#define HID_TEST_SPEED_TIMER       (1000)	//Test timer, it will send one character every 1s
/// HID infomation
#define HID_INFO "\x11\x01\x00\x03"
/// Report ID, Report Type
#define HID_REF1 "\x01\x01"

#if (HID_TEST_SPEED_TIMER)
static evt_timer_t hid_test_timer;
static void hid_test_timer_handler(evt_timer_t *timer, void *param);
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */
static const uint8_t hid_report_map[] = {
    // Report ID 1: Keyboard
    0x05, 0x01,                 // Usage Page (Generic Desktop)
    0x09, 0x06,                 // Usage (Keyboard)
    0xA1, 0x01,                 // Collection (Application)
    0x85, 0x01, 			    //	   Report ID (1)	
    0x05, 0x07,                 //     Usage Page (Key Codes)
    0x19, 0xe0,                 //     Usage Minimum (224)
    0x29, 0xe7,                 //     Usage Maximum (231)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x25, 0x01,                 //     Logical Maximum (1)
    0x75, 0x01,                 //     Report Size (1)
    0x95, 0x08,                 //     Report Count (8)
    0x81, 0x02,                 //     Input (Data, Variable, Absolute)

    0x95, 0x01,                 //     Report Count (1)
    0x75, 0x08,                 //     Report Size (8)
    0x81, 0x01,                 //     Input (Constant) reserved byte(1)

    0x95, 0x05,                 //     Report Count (5)
    0x75, 0x01,                 //     Report Size (1)
    0x05, 0x08,                 //     Usage Page (Page# for LEDs)
    0x19, 0x01,                 //     Usage Minimum (1)
    0x29, 0x05,                 //     Usage Maximum (5)
    0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
    0x95, 0x01,                 //     Report Count (1)
    0x75, 0x03,                 //     Report Size (3)
    0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding

    0x95, 0x06,                 //     Report Count (6)
    0x75, 0x08,                 //     Report Size (8)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x25, 0x65,                 //     Logical Maximum (101)
    0x05, 0x07,                 //     Usage Page (Key codes)
    0x19, 0x00,                 //     Usage Minimum (0)
    0x29, 0x65,                 //     Usage Maximum (101)
    0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)

    0x09, 0x05,                 //     Usage (Vendor Defined)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x26, 0xFF, 0x00,           //     Logical Maximum (255)
    0x75, 0x08,                 //     Report Count (2)
    0x95, 0x02,                 //     Report Size (8 bit)
    0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)

    0xC0,                         // End Collection (Application)

    0x05, 0x0C,                 // Usage Page (Consumer)
    0x09, 0x01,                 // Usage (Consumer Control)
    0xA1, 0x01,                 // Collection (Application)
    0x85, 0x02,  			    // Report ID (2)
    0x19, 0x01,  			    //	   Usage Minimum (Consumer Control) 		
    0x2A, 0xFF, 0x03,   	    //	   Usage Maximum 							 
    0x15, 0x01,  			    //	   Logical Minimum (1) 					
    0x26, 0xFF, 0x03,		    //	   Logical Maximum (1023) 					 
    0x95, 0x01,  			    //     Report Count (1) 						
    0x75, 0x10,  			    //     Report Size (16) 						
    0x81, 0x00,   			    //	   Input (Data,Ary,Abs) 					
    0xC0  	  				    // End Collection 	
};

static uint8_t service_hid_uuid[CS_UUID_16BIT] = {0x12, 0x18};
static uint8_t hid_info_uuid[CS_UUID_16BIT] = {0x4A, 0x2A};
static uint8_t control_point_uuid[CS_UUID_16BIT] = {0x4C, 0x2A};
static uint8_t report_map_uuid[CS_UUID_16BIT] = {0x4B, 0x2A};
static uint8_t protocol_mode_uuid[CS_UUID_16BIT] = {0x4E, 0x2A};
static uint8_t boot_kb_in_report_uuid[CS_UUID_16BIT] = {0x22, 0x2A};
static uint8_t boot_kb_out_report_uuid[CS_UUID_16BIT] = {0x32, 0x2A};
static uint8_t report_uuid[CS_UUID_16BIT] = {0x4D, 0x2A};

static uint16_t att_start_handle;
static uint8_t hid_control_point = 0;
static uint8_t hid_protocol_mode = REPORT_PROTOCOL_MODE;
static uint8_t hid_report1_enabled = 0;
static uint8_t hid_report2_enabled = 0;
static uint8_t hid_report3_enabled = 0;
static uint8_t hid_paired = 0;
static uint8_t hid_encrpted = 0;

static cs_gatt_item_t atts_hid[] = {
    // HID Information
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { hid_info_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ, true, true },

    // HID Control Point
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { control_point_uuid, CS_UUID_16BIT, CS_ATT_PROP_WRITE_CMD, false, true },

    // Report Map
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { report_map_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ, true, true },

    // Protocol Mode
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { protocol_mode_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE_CMD, true, true },

    // Boot Keyboard Input Report
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { boot_kb_in_report_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF, true, true },
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},

    // Boot Keyboard Output Report
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { boot_kb_out_report_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_WRITE_CMD, true, true },

    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { report_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF, true, true},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
    { ob_att_rrd_def, CS_UUID_16BIT, CS_ATT_PROP_READ },

    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { report_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF, true, true},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
    { ob_att_rrd_def, CS_UUID_16BIT, CS_ATT_PROP_READ },

    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { report_uuid, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF, true, true},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
    { ob_att_rrd_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
};

static void service_hid_init(void)
{
    static const cs_gatt_serv_t gatt_service_hid = {
        service_hid_uuid, CS_UUID_16BIT,
        sizeof(atts_hid) / sizeof(atts_hid[0]), atts_hid
    };

    cs_gatts_add_service(&gatt_service_hid, &att_start_handle);
}

static void hid_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GAP_EVT_CONNECTED) {
        hid_report1_enabled = 0;
        hid_report2_enabled = 0;
        hid_report3_enabled = 0;
        hid_paired = 0;
        hid_encrpted = 0;

#if (HID_TEST_SPEED_TIMER)
        evt_timer_set(&hid_test_timer, HID_TEST_SPEED_TIMER, EVT_TIMER_REPEAT, hid_test_timer_handler, NULL);
#endif
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        hid_report1_enabled = 0;
        hid_report2_enabled = 0;
        hid_report3_enabled = 0;
        hid_paired = 0;
        hid_encrpted = 0;

#if (HID_TEST_SPEED_TIMER)
        evt_timer_del(&hid_test_timer);
#endif
    } else if (evt_id == CS_GAP_EVT_PAIRING_REQUEST) {
        hid_paired = 1;
    } else if (evt_id == CS_GAP_EVT_ENCRYPT) {
        hid_encrpted = 1;
    } else if (evt_id == CS_GATTS_EVT_READ_REQ) {
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;
        CS_LOG_DEBUG("%s: READ att_hdl %d\r\n", __func__, req->att_hdl);
        if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_MAP_VAL)) {
            CS_LOG_DEBUG("%s: READ ******* offset %d\r\n", __func__, evt->gatt.read_req.offset);
            const uint8_t *map = &hid_report_map[evt->gatt.read_req.offset];
            int map_len = sizeof(hid_report_map) - evt->gatt.read_req.offset;
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, map, map_len);
        } else if (req->att_hdl == (att_start_handle + HID_IDX_PROTO_MODE_VAL)) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, &hid_protocol_mode, sizeof(hid_protocol_mode));
        } else if (req->att_hdl == (att_start_handle + HID_IDX_HID_INFO_VAL)) {
            CS_LOG_DEBUG("%s: HID_INFO %s\r\n", __func__, bt_hex(HID_INFO, sizeof(HID_INFO)-1));
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_INFO, sizeof(HID_INFO) - 1);
        } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_REP_REF_1)) {
            CS_LOG_DEBUG("%s: HID_REF1 %s\r\n", __func__, bt_hex(HID_REF1, sizeof(HID_REF1)-1));
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_REF1, sizeof(HID_REF1) - 1);
        } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_REP_REF_2)) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_REF1, sizeof(HID_REF1) - 1);
        } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_REP_REF_3)) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_REF1, sizeof(HID_REF1) - 1);
        }
    } else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;
        CS_LOG_DEBUG("%s: write_req att_handle %d\r\n", __func__, req->att_hdl);

        if (att_start_handle < req->att_hdl && req->att_hdl < att_start_handle + HID_IDX_NB) {
            cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);

            if (req->att_hdl == (att_start_handle + HID_IDX_HID_CTNL_PT_VAL)) {
                hid_control_point = req->data[0];
            } else if (req->att_hdl == (att_start_handle + HID_IDX_PROTO_MODE_VAL)) {
                hid_protocol_mode = req->data[0];
            } else if (req->att_hdl == (att_start_handle + HID_IDX_BOOT_KB_IN_REPORT_VAL)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_BOOT_KB_OUT_REPORT_VAL)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_VAL_1)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_VAL_2)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_VAL_3)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_BOOT_KB_IN_REPORT_NTF_CFG)) {
            } else if (req->att_hdl == (att_start_handle + HID_IDX_PROTO_MODE_VAL)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_NTF_CFG_1)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
                hid_report1_enabled = 1;
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_NTF_CFG_2)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
                hid_report2_enabled = 1;
            } else if (req->att_hdl == (att_start_handle + HID_IDX_REPORT_NTF_CFG_3)) {
                CS_LOG_DEBUG("handle %d, data(%s)\r\n", req->att_hdl, bt_hex(req->data, req->len));
                hid_report3_enabled = 1;
            }
        }
    } else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        const cs_gatt_evt_tx_complete_t *req = &evt->gatt.tx_complete;
        CS_LOG_DEBUG("Tx complete: %d\r\n", req->id);
    }
}

uint8_t hid_report_send(uint8_t conn_idx, const struct hid_report_info *report)
{
    uint16_t notify_handle = 0;

    if (report->type == HID_REPORT) {
        notify_handle = att_start_handle + HID_IDX_REPORT_VAL_1 + (report->index * 4);
    } else if (report->type == HID_BOOT_KB_IN_REPORT) {
        notify_handle = att_start_handle + HID_IDX_BOOT_KB_IN_REPORT_VAL;
    } else if (report->type == HID_BOOT_KB_OUT_REPORT) {
        notify_handle = att_start_handle + HID_IDX_BOOT_KB_OUT_REPORT_VAL;
    }

#if 0
    // JUST TEST
    notify_handle = 29;
    static uint8_t xxx = 0;
    xxx++;
#endif

    CS_LOG_DEBUG("%s: data(%s) att_start_handle %d notify_handle %d \r\n",
                 __func__, bt_hex(report->data, report->length),
                 att_start_handle, notify_handle );
    cs_gatts_hvx_t hvx = {
        CS_HANDLE_VALUE_NTF,
        notify_handle,
        report->data,
        report->length,
        notify_handle ,
    };

    uint32_t status = cs_gatts_send_hvx(0, &hvx);
    CS_LOG_DEBUG("%s: status %d\r\n", __func__, status);

    return 0;
}

void app_hid_init(void)
{
    service_hid_init();
    cs_event_callback_reg(hid_event_cb);
}

#if (HID_TEST_SPEED_TIMER)
static uint8_t key_code = 0x04;
static uint8_t key_modifier = 0;
static uint8_t key_player = 0;
static void hid_test_timer_handler(evt_timer_t *timer, void *param)
{
    uint8_t key[8] = {0};
    struct hid_report_info report;
    
    CS_LOG_DEBUG("%s: begin\r\n", __func__);

    if (key_modifier == 0 || key_code<=0x27) {
        // send std key press
        memset(&report, 0, sizeof(struct hid_report_info));
        report.type = HID_REPORT;
        report.index = 0;
        report.length = 8;
        report.data = key;

        key[0] = key_modifier;
        key[2] = key_code;
        hid_report_send(0, &report);

        // send std key release
        memset(&report, 0, sizeof(struct hid_report_info));
        memset(key, 0, 8);
        report.type = HID_REPORT;
        report.index = 0;
        report.length = 8;
        report.data = key;
        hid_report_send(0, &report);

        key_code++;
        if (key_code > 0x27)
        {
            if (key_modifier != 0x02) {
                key_modifier = 0x02;
                key_code = 0x04;
            }
        }
    }
    else {
        memset(&report, 0, sizeof(struct hid_report_info));
        memset(key, 0, 8);
        report.type = HID_REPORT;
        report.index= 1;
        report.length = 8;
        report.data = key;

        if (key_player ==0) {
            key_player = 1;
            // send multim key press
            key[0] = 0xCD;
            key[1] = 0x00; // play
            log_debug("%s: **************** play\r\n", __func__);
            hid_report_send(0, &report);
        }
        else {
            // send multim key press
            key[0] = 0xB7;
            key[1] = 0x00; // stop
            
            key_player = 0;
            key_modifier = 0x00;
            key_code = 0x04;
            log_debug("%s: **************** stop\r\n", __func__);
            hid_report_send(0, &report);

        }

        // release
        memset(&report, 0, sizeof(struct hid_report_info));
        memset(key, 0, 8);
        report.type = HID_REPORT;
        report.index= 1;
        report.length = 8;
        report.data = key;
        hid_report_send(0, &report);
    }

}
#endif

#endif
