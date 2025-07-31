/**
 ****************************************************************************************
 *
 * @file hid.h
 *
 ****************************************************************************************
 */

#ifndef _HID_H_ 
#define _HID_H_ 

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "sdk_config.h"

#if (BLE_HID_DEVICE)


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// HID Service Attributes Indexes
enum
{
    HID_IDX_SVC,

    // HID Information
    HID_IDX_HID_INFO_CHAR,
    HID_IDX_HID_INFO_VAL,

    // HID Control Point
    HID_IDX_HID_CTNL_PT_CHAR,
    HID_IDX_HID_CTNL_PT_VAL,

    // Report Map
    HID_IDX_REPORT_MAP_CHAR,
    HID_IDX_REPORT_MAP_VAL,

    // Protocol Mode
    HID_IDX_PROTO_MODE_CHAR,
    HID_IDX_PROTO_MODE_VAL,

    // Boot Keyboard Input Report
    HID_IDX_BOOT_KB_IN_REPORT_CHAR,
    HID_IDX_BOOT_KB_IN_REPORT_VAL,
    HID_IDX_BOOT_KB_IN_REPORT_NTF_CFG,

    // Boot Keyboard Output Report
    HID_IDX_BOOT_KB_OUT_REPORT_CHAR,
    HID_IDX_BOOT_KB_OUT_REPORT_VAL,

    // Report 1
    HID_IDX_REPORT_CHAR_1,
    HID_IDX_REPORT_VAL_1,
    HID_IDX_REPORT_NTF_CFG_1,
    HID_IDX_REPORT_REP_REF_1,

    // Report 2
    HID_IDX_REPORT_CHAR_2,
    HID_IDX_REPORT_VAL_2,
    HID_IDX_REPORT_NTF_CFG_2,
    HID_IDX_REPORT_REP_REF_2,

    // Report 3
    HID_IDX_REPORT_CHAR_3,
    HID_IDX_REPORT_VAL_3,
    HID_IDX_REPORT_NTF_CFG_3,
    HID_IDX_REPORT_REP_REF_3,

    HID_IDX_NB,
};

enum {
    BOOT_PROTOCOL_MODE,
    REPORT_PROTOCOL_MODE,
};

enum {
    HID_REPORT,
    HID_BOOT_KB_IN_REPORT,
    HID_BOOT_KB_OUT_REPORT,
};

struct hid_report_info {
    uint8_t type;
    uint8_t index;
    uint16_t length;
    uint8_t *data;
};

#if 0
struct ps2_keyboard_msg
{
    uint8_t report_id;
    uint8_t key[8];
};
#endif

void app_hid_init(void);

#endif /* #if (BLE_HID_DEVICE) */

/// @} HID

#endif /* _HID_H_ */
