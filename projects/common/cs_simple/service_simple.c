/* ----------------------------------------------------------------------------
 * Copyright (c) 2020-2030 chipsea Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of chipseaelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -------------------------------------------------------------------------- */

/**
 * @file     service_tspp.c
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */
/*******************************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cs_driver.h"
#include "sdk_config.h"
#include "cs_ble.h"
#include "cs_common_utils.h"
#include "cs_log.h"
#include "service_simple.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*********************************************************************
 * TYPEDEFS
 */

/**@brief GATT Attribute Value. */
typedef struct
{
  uint16_t  len;         
  uint8_t   *p_value;  
} ble_attr_value_t;



/*********************************************************************
 * LOCAL VARIABLES
 */
// Start handle
static uint16_t start_handle_service_simple;
static uint8_t service_simple[CS_UUID_16BIT] = {0xF0, 0xFF};
static uint8_t service_simple_char1[CS_UUID_16BIT] = {0xF1, 0xFF};
static uint8_t service_simple_char2[CS_UUID_16BIT] = {0xF2, 0xFF};
static uint8_t service_simple_char3[CS_UUID_16BIT] = {0xF3, 0xFF};
static uint8_t service_simple_char4[CS_UUID_16BIT] = {0xF4, 0xFF};
static uint8_t service_simple_char5[CS_UUID_16BIT] = {0xF5, 0xFF};
static uint8_t service_simple_char6[CS_UUID_16BIT] = {0xF6, 0xFF};
static uint8_t service_simple_char7[CS_UUID_16BIT] = {0xF7, 0xFF};


static cs_gatt_item_t atts_simple[] = {
    { ob_att_char_def,         CS_UUID_16BIT, CS_ATT_PROP_READ                     },
    { service_simple_char1,    CS_UUID_16BIT, CS_ATT_PROP_READ| CS_ATT_PROP_WRITE  },
    { ob_att_cudd_def,         CS_UUID_16BIT, CS_ATT_PROP_READ  },
    
    { ob_att_char_def,         CS_UUID_16BIT, CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char2,    CS_UUID_16BIT, CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT, CS_ATT_PROP_READ, 0, 0 },
    
    { ob_att_char_def,         CS_UUID_16BIT, CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char3,    CS_UUID_16BIT, CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT, CS_ATT_PROP_READ, 0, 0 },
 
    { ob_att_char_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char4,    CS_UUID_16BIT,  CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ, 0, 0 },
    
    { ob_att_char_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char5,    CS_UUID_16BIT,  CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ, 0, 0 },
    
    { ob_att_char_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char6,    CS_UUID_16BIT,  CS_ATT_PROP_WRITE_CMD | CS_ATT_PROP_NTF, 0, 0 },
    { ob_att_cccd_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ, 0, 0 },
    
    { ob_att_char_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ,                    0, 0 },
    { service_simple_char7,    CS_UUID_16BIT,  CS_ATT_PROP_READ| CS_ATT_PROP_WRITE, 0, 0 },
    { ob_att_cudd_def,         CS_UUID_16BIT,  CS_ATT_PROP_READ, 0, 0 },
};

uint8_t    char1_value[IBEACON_UUID_LEN] = {DEFAULT_IBEACON_UUID};
uint8_t    char1_desp[] = "uuid";
uint16_t   char2_value = DEFAULT_IBEACON_MAJOR;
uint8_t    char2_desp[] = "major";
uint16_t   char3_value = DEFAULT_IBEACON_MINOR;
uint8_t    char3_desp[] = "minor";
uint8_t    char4_value = DEFAULT_IBEACON_POWER;
uint8_t    char4_desp[] = "rssi";
uint8_t    ble_test_cmd[IBEACON_ATT_PKT_LEN] = {0};
uint8_t    char5_desp[] = "test cmd";
uint8_t    ble_ntf_data[MAX_MTU_SIZE] = {0};
uint16_t   char6_ntf_enable = 0;
uint8_t    char6_desp[] = "data up";
uint8_t    ble_rec_data[MAX_MTU_SIZE] = {0};
uint8_t    char7_desp[] = "data down";

ble_attr_value_t  simple_server_char_value[15] ={ 
    {sizeof(char1_value),  char1_value}, 
    {sizeof(char1_desp),   char1_desp}, 
    
    {sizeof(char2_value),  (uint8_t *)&char2_value}, 
    {sizeof(char2_desp),   char2_desp}, 
    
    {sizeof(char3_value),  (uint8_t *)&char3_value}, 
    {sizeof(char3_desp),   char3_desp},   
    
    {sizeof(char4_value),  (uint8_t *)&char4_value}, 
    {sizeof(char4_desp),   char4_desp},
    
    {sizeof(ble_test_cmd),  ble_test_cmd}, 
    {sizeof(char5_desp),   char5_desp},  
    
    {sizeof(ble_ntf_data),    ble_ntf_data}, 
    {sizeof(char6_ntf_enable),  (uint8_t *)&char6_ntf_enable}, 
    {sizeof(char6_desp),   char6_desp},    
    
    {sizeof(ble_rec_data),    ble_rec_data}, 
    {sizeof(char7_desp),   char7_desp},  
};


static uint8_t m_simple_server_mtu;
static uint8_t m_simple_server_send_pending;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8_t simple_server_set_char_data(uint8_t idx, void *value, uint32_t len)
{
    uint8_t  ret = PARAM_SUCCESS;

//    log_debug("%s@%d\n", __func__, __LINE__);
    switch ( idx )
	{
		case IDX_SIMPLE_CHAR1_VAL:
            simple_server_char_value[0].len = len;
			if (len <= IBEACON_UUID_LEN)
			{
                memcpy(simple_server_char_value[0].p_value, value, len);                
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;

		case IDX_SIMPLE_CHAR2_VAL:
            simple_server_char_value[2].len = len;
			if ( len <= IBEACON_MAJOR_LEN ) 
			{
                memcpy(simple_server_char_value[2].p_value, value, len);
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;

		case IDX_SIMPLE_CHAR3_VAL:
            simple_server_char_value[4].len = len;
			if ( len <= IBEACON_MINOR_LEN ) 
			{
                memcpy(simple_server_char_value[4].p_value, value, len);
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;

		case IDX_SIMPLE_CHAR4_VAL:
            simple_server_char_value[6].len = len;
			if ( len <= sizeof ( uint8_t ) ) 
			{
                memcpy(simple_server_char_value[6].p_value, value, len);
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;

		case IDX_SIMPLE_CHAR5_VAL:
            simple_server_char_value[8].len = len;            
			if ( len <= IBEACON_ATT_PKT_LEN)   
			{
                simple_server_char5_pro(len, value);	
                memcpy(simple_server_char_value[8].p_value, value, len);                
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;

        case IDX_SIMPLE_CHAR6_VAL:
            simple_server_char_value[10].len = len;   
			if ( len <= MAX_MTU_SIZE)   
			{
                memcpy(simple_server_char_value[10].p_value, value, len);                
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;
            
        case IDX_SIMPLE_CHAR6_CCCD:
            simple_server_char_value[11].len = len;
			if ( len <= 2)   
			{
                memcpy(simple_server_char_value[11].p_value, value, len);                
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;            
            
		case IDX_SIMPLE_CHAR7_VAL:
            simple_server_char_value[13].len = len; 
			if ( len <= MAX_MTU_SIZE )   
			{
                simple_server_char7_pro(len, value);
                memcpy(simple_server_char_value[13].p_value, value, len);                 
			}
			else
			{
				ret = PARAM_LEN_ERR;
			}
			break;
			
		default:
			ret = INVALIDPARAMETER;
			break;
	}
 
    return ret;
}


uint8_t simple_server_get_char_data(uint8_t idx, void *value, uint16_t *len)
{
    uint8_t  ret = INVALIDPARAMETER;
    uint8_t  index = 0xFF;
	switch ( idx )
	{
		case IDX_SIMPLE_CHAR1_VAL:
            index = 0;		               
			break;
        
		case IDX_SIMPLE_CHAR1_DSP:
            index = 1;		               
			break;  

		case IDX_SIMPLE_CHAR2_VAL:
            index = 2;		               
			break;
        
		case IDX_SIMPLE_CHAR2_DSP:
            index = 3;		               
			break; 
        
		case IDX_SIMPLE_CHAR3_VAL:
            index = 4;		               
			break;
        
		case IDX_SIMPLE_CHAR3_DSP:
            index = 5;		               
			break;
        
		case IDX_SIMPLE_CHAR4_VAL:
            index = 6;		               
			break;
        
		case IDX_SIMPLE_CHAR4_DSP:
            index = 7;		               
			break;
        
		case IDX_SIMPLE_CHAR5_VAL:
            index = 8;		               
			break;
        
		case IDX_SIMPLE_CHAR5_DSP:
            index = 9;		               
			break;
        
		case IDX_SIMPLE_CHAR6_VAL:
            index = 10;		               
			break;
        
		case IDX_SIMPLE_CHAR6_CCCD:
            index = 11;		               
			break;
        
		case IDX_SIMPLE_CHAR6_DSP:
            index = 12;		               
			break;
        
		case IDX_SIMPLE_CHAR7_VAL:
            index = 13;		               
			break;
        
		case IDX_SIMPLE_CHAR7_DSP:
            index = 14;		               
			break;
			
		default:
            *len = 0;
			ret = INVALIDPARAMETER;
			break;
	}
    
    if(index != 0xff)
    {
        *len = simple_server_char_value[index].len;
        if(*len < MAX_MTU_SIZE)
        {
            memcpy(value, simple_server_char_value[index].p_value, simple_server_char_value[index].len ); 
            ret = PARAM_SUCCESS;
        }            
    }

	return ( ret );
}


static void service_simple_init(void)
{
    static const cs_gatt_serv_t att_serv_simple = {
        service_simple, CS_UUID_16BIT,
        sizeof(atts_simple) / sizeof(atts_simple[0]), atts_simple
    };
    cs_gatts_add_service(&att_serv_simple, &start_handle_service_simple);
}

static void service_simple_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GATTS_EVT_READ_REQ) {
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;
        uint8_t idx = req->att_hdl - start_handle_service_simple;
        uint8_t temp_value[MAX_MTU_SIZE];
        uint16_t data_len;
        //CS_LOG_DEBUG("SIMPLE-READ: att_handle(%d), conn_idx(%d)\r\n", req->att_hdl, evt->gatt.conn_idx);
        
        simple_server_get_char_data(idx, temp_value, &data_len);
        cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)temp_value, data_len);
        
    } else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;      
        const uint8_t* data = req->data;
        uint16_t len = req->len;
        uint8_t idx = req->att_hdl - start_handle_service_simple;
        //CS_LOG_DEBUG("SIMPLE-WRITE: att_handle(%d), conn_idx(%d)\r\n", req->att_hdl, evt->gatt.conn_idx);
        
        simple_server_set_char_data(idx, (uint8_t* )data, len); 
        cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);        
        
    } else if (evt_id == CS_GAP_EVT_CONNECTED) {
        char6_ntf_enable = false;
        m_simple_server_mtu = 20;
        #if 0
        cs_gap_data_length_update(evt->gap.conn_idx, 251, 0x2000);
        cs_gap_phys_t phys = {0, 1};
        cs_gap_phy_update(evt->gap.conn_idx, phys, phys);
        #endif
        simple_server_connect_pro();
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        char6_ntf_enable = false;
        simple_server_disconnect_pro();
    }  else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        simple_server_tx_complete_pro();
    } else if (evt_id == CS_BLE_GATT_EVT_MTU_EXCHANGED) {
        m_simple_server_mtu = evt->gatt.mtu_exchanged.mtu - 3;
        CS_LOG_DEBUG("SIMPLE-MTU_EXCHANGED:%d\n",m_simple_server_mtu);
    }
}



/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

void app_service_simple_init(void)
{
    service_simple_init();
    cs_event_callback_reg(service_simple_event_cb);
}


uint8_t check_simple_notify_enable(void)
{
	return char6_ntf_enable;
}

uint8_t simple_mtu_get(void)
{
	return m_simple_server_mtu;
}

uint8_t simple_notify_data_send(uint16_t idx, uint8_t* data, uint16_t len)
{
    uint16_t desc_value;
    uint16_t send_len;
    uint8_t pkg_num = 0, i;
    
    //CS_LOG_DEBUG("send data:len; mtu:%d\n", len, simple_mtu_get());
    if (len > simple_mtu_get()) {
        return PARAM_LEN_ERR;
    }

//	if(check_simple_notify_enable() == 0) {
//        return PARAM_ERR_DISABLED;
//    }

//    extern  uint32_t ob_tx_buffer_count_get(uint8_t *p_count);    
//    ob_tx_buffer_count_get(&pkg_num);
//    if (!pkg_num) {
//        return PARAM_ERR_FULL;
//    }

    //for(i=0;i < pkg_num && len > 0;i++)
	{

        send_len = len;
        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_NTF,
            start_handle_service_simple + idx,      // handle
            data,
            send_len,
            start_handle_service_simple
        };
        if(cs_gatts_send_hvx(0, &hvx))
	    {
            return PARAM_ERR_SEND;
		}
    }
    return PARAM_SUCCESS;
}


__WEAK void simple_server_char5_pro(uint16_t len, uint8_t *value) 
{

}

__WEAK void simple_server_char7_pro(uint16_t len, uint8_t *value) 
{

}

__WEAK void simple_server_disconnect_pro(void) 
{

}

__WEAK void simple_server_connect_pro(void) 
{

}

__WEAK void simple_server_tx_complete_pro(void) 
{

}
/** @} */
