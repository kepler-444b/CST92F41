/*********************************************************************
 * @file ota_protocol.c
 * @version V20221013.1.0
 */

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "ota_protocol.h"
#include "nrf_ota_nvds.h"

#define log_debug CS_LOG_DEBUG
#define log_debug_array_ex CS_LOG_DEBUG_ARRAY_EX

#define NRF_OTA_DEBUG
#if defined(NRF_OTA_DEBUG)
#define dfu_debug(fmt, ...) log_debug("[DFU] " fmt, ## __VA_ARGS__)
#define dfu_debug_array_ex log_debug_array_ex
#else
#define dfu_debug(...)
#define dfu_debug_array_ex(...)
#endif

#if !defined(MAX)
#define MAX(x, y)                           (((x) > (y)) ? (x) : (y))
#define MIN(x, y)                           (((x) < (y)) ? (x) : (y))
#endif


typedef struct {
    uint32_t     app_img_size;		//bin文件总长度Application image size
    uint32_t     app_img_recv_len;	//当前接收bin文件长度 Application receive image size
    uint16_t     cache_recv_len;  //cache buff 接收有效数据长度
    uint16_t     tool_ccitt_crc16;		//Image CRC16 CCITT
    uint16_t     ota_img_crc16;
    uint16_t     app_packet_numb;		//number of notify packets
    uint16_t     app_rec_packet_numb;		//number of notify receive packets
    uint8_t      status;
    uint8_t      img_type;			//Image type: 0x00,No Image;  0x02,Bootloader;  0x04,Application
    uint8_t      packet_buf[32];		//Init Packet
    uint8_t      pkt_buf_len;		//Init Packet length
    ota_pkt_t    pkt_type;            //packet type
}ota_nrf_env_t;

static ota_nrf_env_t env ={0};
__ALIGNED(16) static uint8_t m_cache[DFU_CACHE_BUF_SIZE];

static void dfu_env_init(void)
{
    env.ota_img_crc16 = 0x0000;
    if(dfu_cb_itf.dfu_begin_cb){
        dfu_cb_itf.dfu_begin_cb(0, NULL);
    }
    
}

//static  uint32_t dfu_crc32(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
//{
//    uint32_t crc;
//    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
//    for (uint32_t i = 0; i < size; i++){
//        crc = crc ^ p_data[i];
//        for (uint32_t j = 8; j > 0; j--) crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
//    }
//    return ~crc;
//}


int dfu_set_enable(bool enabled)
{
    if(env.status != DFU_STATUS_LOCKED){
        env.status = enabled?DFU_STATUS_ENABLED:DFU_STATUS_DISABLED;
    }
    return env.status;
}

void dfu_reset(uint8_t state)
{
    dfu_debug("dfu_reset:%d\n", state);
    if(dfu_cb_itf.dfu_end_cb){
        dfu_cb_itf.dfu_end_cb(state, NULL);
    }
}


uint8_t otaFlash_CalculateCRC16(uint16_t crcValue)
{
	uint32_t j, len;
	uint16_t flash_crc16 = 0x0000;
    nrf_ota_nvds_enable_flash();
    for(j=NRF_OTA_BIN_ADDR;j<NRF_OTA_BIN_ADDR+env.app_img_size;)
    {
        // TODO: If it takes too long to calculate the CRC, you need to add an action here to restart the watchdog timer
        len = MIN(DFU_CACHE_BUF_SIZE, NRF_OTA_BIN_ADDR+env.app_img_size-j);
        nrf_ota_nvds_get_flash(j, &len, m_cache);
        flash_crc16 = cs_crc16_ccitt(flash_crc16, m_cache, len);
        j += len;
    }
	
	dfu_debug("CheckCRC=%x,DownCRC=%x,SetCRC=%x\r\n",flash_crc16, env.ota_img_crc16,crcValue);
	if(flash_crc16 == crcValue && flash_crc16 == env.ota_img_crc16) {
		return DFU_SUCCESS;
	}
	return DFU_INVALID_CODE;
}


static void write_image_data(uint8_t *data, uint32_t len)
{
    uint32_t pos = 0;
    while(pos < len){
        // Copy data to m_cache
        uint32_t remain_data = len - pos;		//当次接收
        uint32_t remain_cache = DFU_CACHE_BUF_SIZE - env.cache_recv_len;  //剩余cache buff长度
        uint32_t remain_image = env.app_img_size - env.app_img_recv_len;  //剩余bin总长度
        uint32_t copy2cache_len = MIN(MIN(remain_data, remain_cache), remain_image);

//		log_debug("remain_data:%02x,remain_cache:%02x,remain_image:%02x,copy2cache_len:%02x\r\n",remain_data,remain_cache,remain_image,copy2cache_len);
        memcpy(&m_cache[env.cache_recv_len], &data[pos], copy2cache_len);
        pos += copy2cache_len;
        env.cache_recv_len += copy2cache_len;
        env.app_img_recv_len += copy2cache_len;

        // Write m_cache to flash
        if(env.cache_recv_len == DFU_CACHE_BUF_SIZE || env.app_img_recv_len == env.app_img_size){
            nrf_ota_nvds_enable_flash();
            uint32_t write_addr = NRF_OTA_BIN_ADDR + env.app_img_recv_len - env.cache_recv_len;
#if FLASH_ERASE_SIZE
            uint32_t erase_addr = (write_addr + FLASH_ERASE_SIZE - 1) & ~(FLASH_ERASE_SIZE - 1);
            for(;erase_addr < write_addr + env.cache_recv_len;erase_addr += FLASH_ERASE_SIZE){
                dfu_debug("Erasing 0x%08X-0x%08X (%s)\n", erase_addr, erase_addr+FLASH_ERASE_SIZE, env.cmd_img_info->describe);
                nrf_ota_nvds_erase_flash(erase_addr, FLASH_ERASE_SIZE);
            }
#endif
            nrf_ota_nvds_put_flash(write_addr, env.cache_recv_len, m_cache);
            env.ota_img_crc16 = cs_crc16_ccitt(env.ota_img_crc16, m_cache, env.cache_recv_len);
            env.cache_recv_len = 0;
        }
    }
}



void otaProtocol_ProcessControlEvt(uint8_t *dat, uint32_t len, dfu_response_t *response)
{
	uint8_t opcode;
	opcode = dat[0];
    response->rsp_code = OTA_CODE_RESPONSE;
    response->opcode = opcode;
    response->result = OTA_RESP_VAL_SUCCESS;
    response->length = DFU_RESP_SIZE_NO_EXT_DATA;
	switch(opcode) {
		case OTA_CODE_START_OTA:
            response->length = DFU_RESP_SIZE_NO_DATA;
            env.pkt_type = PKT_TYPE_START;
            env.img_type = dat[1];
            dfu_env_init();                     //updating connection parameter 
            dfu_debug("mini OTA,ImageType=%d\r\n",env.img_type);

			break;
		case OTA_CODE_RECEIVE_INIT:
			env.pkt_type = PKT_TYPE_INIT;
			if(dat[1] == 0x00) {		//Receive Init Packet
                response->length = DFU_RESP_SIZE_NO_DATA;
				env.pkt_buf_len = 0;
				memset(env.packet_buf,0,sizeof(env.packet_buf));
			}
			else if(dat[1] == 0x01) {	//Init Packet Complete
				if(env.pkt_buf_len >= 2) {
                    response->length = DFU_RESP_SIZE_NO_DATA;
					env.tool_ccitt_crc16 = (env.packet_buf[env.pkt_buf_len-2]<<0)|(env.packet_buf[env.pkt_buf_len-1]<<8);
					dfu_debug("Image CRC=%04x\r\n",env.tool_ccitt_crc16);
				}
				else {
                    response->result = OTA_RESP_VAL_OPER_FAILED;
				}
			}
			else {	//Invalid State
                response->result = OTA_RESP_VAL_INVALID_STATE;
			}
			dfu_debug("Receive Init\r\n");
			break;
		case OTA_CODE_RECEIVE_FW:
            response->length = DFU_RESP_SIZE_NO_DATA;
			env.pkt_type = PKT_TYPE_FIRMWARE_DATA;
            response->length = DFU_RESP_SIZE_NO_DATA;
			dfu_debug("Receive Fw\r\n");
			break;
		case OTA_CODE_VALIDATE:
			env.pkt_type = PKT_TYPE_INVALID;
			if(otaFlash_CalculateCRC16(env.tool_ccitt_crc16)) {
                nrf_ota_nvds_enable_mbr();
                dfu_image_mbr_info info = {
                                    NRF_OTA_BIN_ADDR,
                                    env.app_img_size,
                                    env.tool_ccitt_crc16,
                                };
                dfu_debug("mbr_info=%04x  %04x  %04x\r\n",info.address,info.crc16,info.length);
                nrf_ota_nvds_put_mbr(IMAGE_TYPE_APP, sizeof(info), &info);
                 uint32_t len;                                  
                nrf_ota_nvds_get_mbr(IMAGE_TYPE_APP, &len, &info);         
                dfu_debug("get mbr_info=%04x  %04x  %04x\r\n",info.address,info.crc16,info.length);                            
				dfu_debug("Validate OK\r\n");
			}
			else {
                response->result = OTA_RESP_VAL_CRC_ERROR;
				dfu_debug("Validate Fail\r\n");
			}
			break;
		case OTA_CODE_ACTIVATE_N_RESET:
            response->length = DFU_RESP_SIZE_NO_DATA;
			dfu_debug("Activate and reset\r\n");
			env.pkt_type = PKT_TYPE_INVALID;
			env.app_packet_numb = 0xFFFF;
			env.pkt_buf_len = 0;
			env.app_img_recv_len = 0;
			env.app_rec_packet_numb = 0;
            dfu_reset(DFU_UPDATE_ST_SUCCESS);
			break;
		case OTA_CODE_SYS_RESET:
            response->length = DFU_RESP_SIZE_NO_DATA;
			dfu_debug("Sys reset\r\n");
			env.pkt_type = PKT_TYPE_INVALID;
			env.app_packet_numb = 0xFFFF;
			env.pkt_buf_len = 0;
			env.app_img_recv_len = 0;
			env.app_rec_packet_numb = 0;

			dfu_reset(DFU_UPDATE_ST_SUCCESS);
			break;
		case OTA_CODE_IMAGE_SIZE_REQ:
			env.pkt_type = PKT_TYPE_INVALID;
			if(env.img_type == OTA_IMAGE_TYPE_BOOTLOADER) {
                response->length =DFU_RESP_SIZE_IMG_SIZE;
			}
			else if(env.img_type == OTA_IMAGE_TYPE_APPLICATION) {
                response->length =DFU_RESP_SIZE_IMG_SIZE;
                response->data.img_size = env.app_img_recv_len;
			}
			else {
                response->result = OTA_RESP_VAL_INVALID_STATE;
			}
			dfu_debug("Image size Req\r\n");
			break;
		case OTA_CODE_PKT_RCPT_NOTIF_REQ:
            response->length = DFU_RESP_SIZE_NO_DATA;
			env.pkt_type = PKT_TYPE_INVALID;
			env.app_packet_numb = (dat[1]<<0)|(dat[2]<<8);
			env.app_rec_packet_numb = 0;
			dfu_debug("OTA RCPT number:%d\r\n", env.app_packet_numb);
			break;
		default:
            response->length = DFU_RESP_SIZE_NO_DATA;
			break;
	}
}


void otaProtocol_ProcessDataEvt(uint8_t *dat, uint32_t len,dfu_response_t *response)
{
    response->rsp_code = OTA_CODE_RESPONSE;
    response->result = OTA_RESP_VAL_SUCCESS;
    response->length = DFU_RESP_SIZE_NO_EXT_DATA;
    response->opcode = env.pkt_type;
	switch(env.pkt_type) {
		case PKT_TYPE_START:
			if(len == 12) {
				env.app_img_size = (dat[8]<<0)|(dat[9]<<8)|(dat[10]<<16)|(dat[11]<<24);
				
				env.app_packet_numb = 0xFFFF;
				env.pkt_buf_len = 0;
				env.app_img_recv_len = 0;
				env.app_rec_packet_numb = 0;
				dfu_debug("AppSize=%d\r\n", env.app_img_size);
#if !FLASH_ERASE_SIZE
                // Erase flash for new image
                nrf_ota_nvds_enable_flash();
                nrf_ota_nvds_erase_flash(NRF_OTA_BIN_ADDR, env.app_img_size);
#endif
			}
			else {	//data length is error
                response->result = OTA_RESP_VAL_NOT_SUPPORTED;
			}
			break;
		case PKT_TYPE_INIT:
            dfu_debug("PKT_TYPE_INIT");
			if(env.pkt_buf_len + len <= sizeof(env.packet_buf)) {
				memcpy(env.packet_buf+env.pkt_buf_len, dat, len);
				env.pkt_buf_len += len;
			}
			else {
				response->result = OTA_RESP_VAL_NOT_SUPPORTED;
			}
			break;
		case PKT_TYPE_FIRMWARE_DATA:
//			dfu_debug("-");
            response->length = DFU_RESP_SIZE_NO_DATA;
			env.app_rec_packet_numb++;
			if(env.img_type == OTA_IMAGE_TYPE_BOOTLOADER) {
   
                response->rsp_code = OTA_CODE_RESPONSE;
                response->result = OTA_RESP_VAL_NOT_SUPPORTED;
                response->length = DFU_RESP_SIZE_NO_EXT_DATA;
                response->opcode = PKT_TYPE_FIRMWARE_DATA;
				
			}
			else if(env.img_type == OTA_IMAGE_TYPE_APPLICATION) {
				write_image_data(dat, len);
				if(env.app_rec_packet_numb >= env.app_packet_numb) {
					env.app_rec_packet_numb = 0;
                    response->length =DFU_RESP_SIZE_PACKET_NUM;
                    response->rsp_code = OTA_CODE_PKT_RCPT_NOTIF;
                    memcpy(&response->opcode,(uint8_t *)&env.app_img_recv_len,4);
//                    dfu_debug("appImageRecSize=%d\r\n", env.app_img_recv_len);
				}
				if(env.app_img_recv_len >= env.app_img_size) {
                    response->rsp_code = OTA_CODE_RESPONSE;
                    response->result = OTA_RESP_VAL_SUCCESS;
                    response->length = DFU_RESP_SIZE_NO_EXT_DATA;
                    response->opcode = PKT_TYPE_FIRMWARE_DATA;
                    dfu_debug("env.app_img_recv_len=%d\r\n", env.app_img_recv_len);
				}
			}
			break;
		default:
            response->length = DFU_RESP_SIZE_NO_DATA;
			break;
	}
}


