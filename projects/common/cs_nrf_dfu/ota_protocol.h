/*********************************************************************
 * @file ota_protocol.h
 * @version V20221013.1.0
 */
 
#ifndef __NRF_OTA_H__
#define __NRF_OTA_H__
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include "nrf_ota_nvds.h"

/*
    0x01BFDF55 | Protocol Version(2 Bytes) | RFU(2Bytes) | Ctrl bitmap | Image Num | N × 『|Type|RFU|Version|SIZE|』| Reserved |
*/


#define OTA_IMAGE_TYPE_BOOTLOADER		2
#define OTA_IMAGE_TYPE_APPLICATION		4



enum{
	OTA_CODE_START_OTA				= 1,		/**< Value of the Op code field for 'Start OTA' command.*/
	OTA_CODE_RECEIVE_INIT			= 2,		/**< Value of the Op code field for 'Initialize OTA parameters' command.*/
	OTA_CODE_RECEIVE_FW				= 3,		/**< Value of the Op code field for 'Receive firmware image' command.*/
	OTA_CODE_VALIDATE				= 4,		/**< Value of the Op code field for 'Validate firmware' command.*/
	OTA_CODE_ACTIVATE_N_RESET		= 5,		/**< Value of the Op code field for 'Activate & Reset' command.*/
	OTA_CODE_SYS_RESET				= 6,		/**< Value of the Op code field for 'Reset System' command.*/
	OTA_CODE_IMAGE_SIZE_REQ			= 7,		/**< Value of the Op code field for 'Report received image size' command.*/
	OTA_CODE_PKT_RCPT_NOTIF_REQ		= 8,		/**< Value of the Op code field for 'Request packet receipt notification.*/
	                                            /**< Value of the Op code field for 'Response.*/
	OTA_CODE_RESPONSE				= 16,		/**< Value of the Op code field for 'Packets Receipt Notification'.*/
	OTA_CODE_PKT_RCPT_NOTIF			= 17,		
};

/**@brief Packet type enumeration.
 */
typedef enum
{
    PKT_TYPE_INVALID,                                                                                /**< Invalid packet type. Used for initialization purpose.*/
    PKT_TYPE_START,                                                                                  /**< Start packet.*/
    PKT_TYPE_INIT,                                                                                   /**< Init packet.*/
    PKT_TYPE_FIRMWARE_DATA                                                                           /**< Firmware data packet.*/
} ota_pkt_t;

/**@brief   OTA Response value type.
 */
typedef enum
{
    OTA_RESP_VAL_SUCCESS = 1,                                       /**< Success.*/
    OTA_RESP_VAL_INVALID_STATE,                                     /**< Invalid state.*/
    OTA_RESP_VAL_NOT_SUPPORTED,                                     /**< Operation not supported.*/
    OTA_RESP_VAL_DATA_SIZE,                                         /**< Data size exceeds limit.*/
    OTA_RESP_VAL_CRC_ERROR,                                         /**< CRC Error.*/
    OTA_RESP_VAL_OPER_FAILED                                        /**< Operation failed.*/
} ota_respon_val_t;

///////////////////////////////////////////////////////////////////////////////

enum{
    DFU_UPDATE_ST_SUCCESS = 0,
    DFU_UPDATE_ST_DISCONNECT,
    DFU_UPDATE_ST_FAILED,
    DFU_UPDATE_ST_TIMEOUT,
};

enum{
    DFU_STATUS_ENABLED = 0,
    DFU_STATUS_DISABLED,
    DFU_STATUS_LOCKED,
};

enum{
    DFU_CTRL_BIT_SIGN     = 1<<0,
    DFU_CTRL_BIT_MORE_IMG = 1<<1,
};



typedef void(*dfu_indicate_cb_t)(uint8_t status, void *p);
typedef uint8_t (*dfu_user_check_cb_t)(uint8_t img_type, uint32_t img_size, uint32_t img_version);

typedef struct {
    const dfu_indicate_cb_t dfu_begin_cb;
    const dfu_indicate_cb_t dfu_prog_cb;
    const dfu_indicate_cb_t dfu_end_cb;
    const dfu_user_check_cb_t dfu_user_check_cb;
}dfu_cb_itf_t;

extern const dfu_cb_itf_t dfu_cb_itf;

#define DFU_DATA_MAX_SIZE            0x10000000

#define DFU_CACHE_BUF_SIZE           128 // 作为写入flash的cache buffer，需要16字节对齐

#define DFU_INVALID_CODE             0x00
#define DFU_SUCCESS                  0x01
#define DFU_OPCODE_NOT_SUPPORT       0x02
#define DFU_INVALID_PARAMETER        0x03
#define DFU_INSUFFICIENT_RESOURCES   0x04
#define DFU_INVALID_OBJECT           0x05
#define DFU_UNSUPPORTED_TYPE         0x07
#define DFU_OPERATION_NOT_PERMITTED  0x08
#define DFU_OPERATION_FAILED         0x0A
#define DFU_VERSION_NOT_MATCH        0x41

// DFU_RESP_SIZE
#define DFU_RESP_SIZE_NO_DATA      0
#define DFU_RESP_SIZE_NO_EXT_DATA  3
#define DFU_RESP_SIZE_IMG_SIZE     7
#define DFU_RESP_SIZE_PACKET_NUM   5



union ota_rsp_data
{
    uint8_t *data;
    uint32_t img_size;
};

typedef struct
{
    uint8_t length; // @ref DFU_RESP_SIZE
    uint8_t rsp_code;
    uint8_t opcode;
    uint8_t result;
    union ota_rsp_data data;
} dfu_response_t;


struct dfu_app_desc_data{
    uint32_t rsp_length;
    const char *app_describe;
};
struct dfu_version_data{
    uint32_t address;
    uint32_t size;
    uint32_t version;
};

typedef union
{
    struct dfu_app_desc_data desc_data; // response read verison
    struct dfu_version_data version_data; // notify version info
} dfu_version_t;

#define dfu_assert(p)    co_assert(p)

void dfu_reset(uint8_t reason); // reason: disconnect/timeout
int dfu_set_enable(bool enabled); // default: DFU_STATUS_ENABLED
void otaProtocol_ProcessDataEvt(uint8_t *dat, uint32_t len,dfu_response_t *response);
void otaProtocol_ProcessControlEvt(uint8_t *dat, uint32_t len, dfu_response_t *response);
#endif /* __NRF_OTA_H__ */
