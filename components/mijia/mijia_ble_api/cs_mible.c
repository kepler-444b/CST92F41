/**
 ****************************************************************************************
 *
 * @file cs_mible.c
 * @brief
 * @date 2024-03-25
 * @author chenchunyu
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdlib.h>
#include <string.h>

#include "mible_api.h"
#include "mible_port.h"
#include "mible_type.h"
#include "mible_log.h"
#include "mible_memory.h"
#include "mi_config.h"

#include "aes.h"
#include "crc32.h"
#include "evt_timer.h"
#include "gatt_dfu/mible_dfu_main.h"
#include "mbr.h"
#include "nvds.h"
#include "cs_ble_mem.h"
#include "cs_ble.h"
#include "cs_ble_gap.h"
#include "cs_ble_gatt.h"
#include "drv_i2c.h"
#include "drv_pmu.h"

uint32_t mi_app_len = 0;
uint32_t mi_app_crc = 0;
// Timer
#define MAX_TIMER_MI 10
typedef struct
{
    evt_timer_t timer;
    mible_timer_handler cb;
    uint8_t mode;
    void *p;
} mi_timer_t;

struct att_val_list
{
    void *next;
    uint16_t handle;
    uint8_t *value;
    uint16_t length;
};

struct att_auth_list
{
    void *next;
    uint16_t handle;
    bool rd_author;
    bool wr_author;
};

mi_timer_t mi_timer[MAX_TIMER_MI];

static uint8_t rand_addr[6];
static uint8_t adv_data[31] = {0};
static uint8_t scan_rsp_data[31] = {0};
static cs_adv_param_t g_adv_param;
static cs_data_t g_adv_data = {
    .len = 0,
    .data = adv_data,
};
static cs_data_t g_scan_rsp_data = {
    .len = 0,
    .data = scan_rsp_data,
};
static cs_gatt_serv_t *g_serv = NULL;
static struct att_val_list *att_value_list_header;
static struct att_auth_list *att_prop_list_header;

static struct att_auth_list *att_prop_list_find_by_handle(uint16_t handle)
{
    struct att_auth_list *att = att_prop_list_header;
    while (att)
    {
        if (att->handle == handle)
        {
            return att;
        }
        att = att->next;
    }
    return NULL;
}

static void att_auth_list_add(uint16_t handle, bool rd_author, bool wr_author)
{
    if (att_prop_list_find_by_handle(handle) == NULL)
    {
        struct att_auth_list *att = mible_malloc(sizeof(struct att_auth_list));
        if (att)
        {
            att->next = att_prop_list_header;
            att->handle = handle;
            att->rd_author = rd_author;
            att->wr_author = wr_author;
            att_prop_list_header = att;
        }
    }
}

static void att_auth_list_clear(void)
{
    struct att_auth_list *current = att_prop_list_header;
    struct att_auth_list *next = NULL;
    while (current != NULL)
    {
        next = current->next;
        mible_free(current);
        current = next;
    }
}

static struct att_val_list *att_val_list_find_by_handle(uint16_t handle)
{
    struct att_val_list *att = att_value_list_header;
    while (att)
    {
        if (att->handle == handle)
        {
            return att;
        }
        att = att->next;
    }
    return NULL;
}

static mible_status_t att_val_list_add(uint16_t handle, const uint8_t *value, uint16_t length)
{
    struct att_val_list *att = att_val_list_find_by_handle(handle);
    if (att == NULL)
    {
        att = mible_malloc(sizeof(struct att_val_list));
        if (att)
        {
            att->next = att_value_list_header;
            att->handle = handle;
            att->value = mible_malloc(length);
            att->length = length;
            memcpy(att->value, value, length);
            att_value_list_header = att;
            return MI_SUCCESS;
        }
    }
    else
    {
        if (att->length != 0)
        {
            mible_free(att->value);
        }
        att->value = mible_malloc(length);
        att->length = length;
        memcpy(att->value, value, length);
        return MI_SUCCESS;
    }

    return MIBLE_ERR_ATT_INVALID_ATT_HANDLE;
}

static void att_value_list_clear(void)
{
    struct att_val_list *current = att_value_list_header;
    struct att_val_list *next = NULL;
    while (current != NULL)
    {
        if (current->value != NULL)
        {
            mible_free(current->value);
            current->length = 0;
        }
        next = current->next;
        mible_free(current);
        current = next;
    }
}

static mible_status_t att_value_list_get(uint16_t handle, uint8_t *p_value, uint8_t *p_len)
{
    struct att_val_list *att = att_val_list_find_by_handle(handle);
    *p_len = 0;

    if (att)
    {
        if (att->value != NULL && att->length != 0)
        {
            *p_len = att->length;
            memcpy(p_value, att->value, att->length);
            return MI_SUCCESS;
        }
        return MI_ERR_INVALID_LENGTH;
    }

    return MIBLE_ERR_ATT_INVALID_ATT_HANDLE;
}

/**
 * @brief co_generate_random_32bit()
 *
 * @return
 **/
static uint32_t co_generate_random_32bit(void)
{
    return (uint32_t)rand();
}

/**
 * @brief co_generate_random()
 *
 * @param[in] rand
 * @param[in] bytes
 *
 * @return
 **/
static void co_generate_random(void *rand, unsigned bytes)
{
    uint32_t tmp = 0;
    uint8_t *prand = rand;
    unsigned align_bytes = bytes & ~0x3;
    unsigned left_bytes = bytes & 0x3;

    for (int i = 0; i < align_bytes; i += 4)
    {
        tmp = co_generate_random_32bit();
        memcpy(prand, &tmp, 4);
        prand += 4;
    }

    if (left_bytes)
    {
        tmp = co_generate_random_32bit();
        memcpy(prand, &tmp, left_bytes);
    }
}

static void gatt_event_cb(uint16_t evt_id, const cs_ble_evt_t *p_evt)
{
    mible_gap_evt_param_t mi_param;

    memset(&mi_param, 0, sizeof(mi_param));
    if (evt_id == CS_GAP_EVT_CONNECTED)
    {

        mi_param.conn_handle = p_evt->gap.conn_idx;
        mi_param.connect.conn_param.min_conn_interval = p_evt->gap.connected.conn_params.conn_intv;
        mi_param.connect.conn_param.max_conn_interval = p_evt->gap.connected.conn_params.conn_intv;
        mi_param.connect.conn_param.slave_latency = p_evt->gap.connected.conn_params.latency_max;
        mi_param.connect.conn_param.conn_sup_timeout = p_evt->gap.connected.conn_params.timeout;
        mi_param.connect.type = (0 == p_evt->gap.connected.peer_addr.addr_type) ? MIBLE_ADDRESS_TYPE_PUBLIC : MIBLE_ADDRESS_TYPE_RANDOM;
        memcpy(mi_param.connect.peer_addr, p_evt->gap.connected.peer_addr.addr, 6);
        mi_param.connect.role = (0 == p_evt->gap.connected.role) ? MIBLE_GAP_CENTRAL : MIBLE_GAP_PERIPHERAL;
        mible_gap_event_callback(MIBLE_GAP_EVT_CONNECTED, &mi_param);
    }
    else if (evt_id == CS_GAP_EVT_DISCONNECTED)
    {
        mi_param.conn_handle = p_evt->gap.conn_idx;
        if (p_evt->gap.disconnected.reason == 0x08 /*CO_ERROR_CON_TIMEOUT*/)
        {
            mi_param.disconnect.reason = CONNECTION_TIMEOUT;
        }
        else if (p_evt->gap.disconnected.reason == 0x13 /*CO_ERROR_REMOTE_USER_TERM_CON*/)
        {
            mi_param.disconnect.reason = REMOTE_USER_TERMINATED;
        }
        else if (p_evt->gap.disconnected.reason == 0x16 /*CO_ERROR_CON_TERM_BY_LOCAL_HOST*/)
        {
            mi_param.disconnect.reason = LOCAL_HOST_TERMINATED;
        }

        mible_gap_event_callback(MIBLE_GAP_EVT_DISCONNECT, &mi_param);
    }
    else if (evt_id == CS_GAP_EVT_CONN_PARAMS_UPDATE)
    {
        mi_param.update_conn.conn_param.min_conn_interval = p_evt->gap.conn_params_update.conn_intv_min;
        mi_param.update_conn.conn_param.max_conn_interval = p_evt->gap.conn_params_update.conn_intv_max;
        mi_param.update_conn.conn_param.slave_latency = p_evt->gap.conn_params_update.latency_max;
        mi_param.update_conn.conn_param.conn_sup_timeout = p_evt->gap.conn_params_update.timeout;

        mible_gap_event_callback(MIBLE_GAP_EVT_CONN_PARAM_UPDATED, &mi_param);
    }
    else if (evt_id == CS_GAP_EVT_ADV_STATE_CHANGED)
    {
        MI_LOG_INFO("["__FILE__"][Line: %d]: %s, adv state is changed, state: %d\n", __LINE__, __func__, p_evt->gap.adv_state_changed.state);
    }
    else if (evt_id == CS_GATTS_EVT_READ_REQ)
    {
        uint8_t len = 0;
        uint8_t data[256] = {0};
        if (att_value_list_get(p_evt->gatt.read_req.att_hdl, data, &len) == MI_SUCCESS)
        {
            cs_gatts_read_response(p_evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, data, len);
        }
        else
        {
            cs_gatts_read_response(p_evt->gatt.conn_idx, CS_BLE_GATT_ERR_INVALID_HANDLE, NULL, 0);
        }
    }
    else if (evt_id == CS_GATTS_EVT_WRITE_REQ)
    {
        mible_gatts_evt_param_t evt;

        att_val_list_add(p_evt->gatt.write_req.att_hdl, p_evt->gatt.write_req.data, p_evt->gatt.write_req.len);
        evt.conn_handle = p_evt->gatt.conn_idx;
        evt.write.offset = 0;
        evt.write.len = p_evt->gatt.write_req.len;
        evt.write.value_handle = p_evt->gatt.write_req.att_hdl;
        evt.write.data = (uint8_t *)p_evt->gatt.write_req.data;
        struct att_auth_list *att = att_prop_list_find_by_handle(p_evt->gatt.write_req.att_hdl);
        if (att && att->wr_author)
        {
            mible_gatts_event_callback(MIBLE_GATTS_EVT_WRITE_PERMIT_REQ, &evt);
        }
        else
        {
            mible_gatts_event_callback(MIBLE_GATTS_EVT_WRITE, &evt);
        }
    }
    else if (evt_id == CS_GAP_EVT_HCI_ERROR)
    {
        MI_LOG_INFO("["__FILE__
                    "][Line: %d]: %s, HCI_ERROR!!!, hci_opcode: 0x%x, status: 0x%02x\n",
                    __LINE__, __func__, p_evt->gap.hci_error.hci_opcode, p_evt->gap.hci_error.status);
    }
    else
    {
        //        MI_LOG_INFO("["__FILE__"][Line: %d]: %s, evt_id = %d, hci_opcode = 0x%x, status = 0x%x\n",
        //                     __LINE__, __func__, evt_id, p_evt->gap.hci_error.hci_opcode, p_evt->gap.hci_error.status);
    }
}

// event and callback
mible_status_t mible_gap_connect(mible_gap_scan_param_t scan_param,
                                 mible_gap_connect_t conn_param)
{
    return MI_SUCCESS;
}

// gap
mible_status_t mible_gap_address_get(mible_addr_t mac)
{
    cs_gap_addr_get(CS_ADV_ADDR_TYPE_RANDOM, rand_addr);
    memcpy(mac, rand_addr, sizeof(rand_addr));
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, addr = [ %02x:%02x:%02x:%02x:%02x:%02x ]\n",
                __LINE__, __func__, rand_addr[5], rand_addr[4], rand_addr[3], rand_addr[2], rand_addr[1], rand_addr[0]);
    return MI_SUCCESS;
}

mible_status_t mible_gap_adv_start(mible_gap_adv_param_t *p_param)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    uint8_t ch_map = 0;
    uint16_t adv_prop = 0;
    mible_status_t res = MI_SUCCESS;

    if (p_param == NULL)
    {
        res = MI_ERR_INVALID_PARAM;
        return res;
    }

    if (!(p_param->ch_mask.ch_37_off & 1))
    {
        ch_map |= CS_ADV_CH_37;
    }
    if (!(p_param->ch_mask.ch_38_off & 1))
    {
        ch_map |= CS_ADV_CH_38;
    }
    if (!(p_param->ch_mask.ch_39_off & 1))
    {
        ch_map |= CS_ADV_CH_39;
    }
    if (p_param->adv_type == MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED)
    {
        adv_prop = CS_ADV_PROP_LEGACY_IND;
    }
    else if (p_param->adv_type == MIBLE_ADV_TYPE_SCANNABLE_UNDIRECTED)
    {
        adv_prop = CS_ADV_PROP_LEGACY_SCAN_IND;
    }
    else if (p_param->adv_type == MIBLE_ADV_TYPE_NON_CONNECTABLE_UNDIRECTED)
    {
        adv_prop = CS_ADV_PROP_LEGACY_NONCONN_IND;
    }
    g_adv_param.own_addr_type = CS_ADV_ADDR_TYPE_RANDOM;
    g_adv_param.prim_phy = CS_ADV_PHY_1M;
    g_adv_param.secd_phy = CS_ADV_PHY_1M;
    g_adv_param.tx_pwr = 0;
    g_adv_param.timeout = 0;
    g_adv_param.filter_policy = CS_ADV_FILTER_NONE;
    g_adv_param.local_addr = rand_addr;
    g_adv_param.peer_addr = NULL;
    g_adv_param.prim_ch_map = ch_map;
    g_adv_param.prim_intv_min = p_param->adv_interval_min;
    g_adv_param.prim_intv_max = p_param->adv_interval_max;
    g_adv_param.adv_properties = adv_prop;

    if (cs_gap_adv_start(0, &g_adv_param, &g_adv_data, &g_scan_rsp_data) != CS_BLE_ERROR_NO_ERR)
    {
        res = MI_ERR_INVALID_STATE;
    }

    return res;
}

mible_status_t mible_gap_adv_data_set(uint8_t const *p_data,
                                      uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    if (dlen)
    {
        g_adv_data.len = dlen;
        memset(g_adv_data.data, 0, 31);
        memcpy(g_adv_data.data, p_data, dlen);
    }
    if (srdlen)
    {
        g_scan_rsp_data.len = srdlen;
        memset(g_scan_rsp_data.data, 0, 31);
        memcpy(g_scan_rsp_data.data, p_sr_data, srdlen);
    }

    return MI_SUCCESS;
}

mible_status_t mible_gap_adv_stop(void)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    return cs_gap_adv_stop(0);
}

mible_status_t mible_gap_disconnect(uint16_t conn_handle)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    mible_status_t res = MI_SUCCESS;

    if (cs_gap_disconnect(conn_handle, 0x13 /* Remote user terminate connect */) != CS_BLE_ERROR_NO_ERR)
    {
        res = MI_ERR_INVALID_STATE;
    }

    return res;
}

mible_status_t mible_gap_update_conn_params(uint16_t conn_handle,
                                            mible_gap_conn_param_t conn_params)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    mible_status_t res = MI_SUCCESS;
    cs_gap_conn_params_t cs_param;

    cs_param.conn_intv = conn_params.max_conn_interval;
    cs_param.latency_max = conn_params.slave_latency;
    cs_param.timeout = conn_params.conn_sup_timeout;

    if (cs_gap_conn_param_update(conn_handle, &cs_param) != CS_BLE_ERROR_NO_ERR)
    {
        res = MI_ERR_INVALID_STATE;
    }

    return res;
}

mible_status_t mible_user_timer_create(void **p_timer_id,
                                       mible_timer_handler timeout_handler, mible_timer_mode mode)
{
    return mible_timer_create(p_timer_id, timeout_handler, mode);
}

mible_status_t mible_timer_create(void **p_timer_id,
                                  mible_timer_handler timeout_handler, mible_timer_mode mode)
{
    uint8_t i = 0;

    for (i = 0; i < MAX_TIMER_MI; i++)
    {
        if (&mi_timer[i] == *p_timer_id)
        {
            return MI_SUCCESS;
        }
    }

    for (i = 0; i < MAX_TIMER_MI; i++)
    {
        if (mi_timer[i].cb == NULL)
        {
            break;
        }
    }

    if (i >= MAX_TIMER_MI)
    {
        return MI_ERR_NO_MEM;
    }

    *p_timer_id = &mi_timer[i];
    mi_timer[i].cb = timeout_handler;
    mi_timer[i].mode = mode;

    return MI_SUCCESS;
}

mible_status_t mible_timer_delete(void *timer_id)
{
    mible_timer_stop(timer_id);
    memset(timer_id, 0, sizeof(mi_timer_t));

    return MI_SUCCESS;
}

static void timer_callback(evt_timer_t *timer, void *param)
{
    mi_timer_t *p_mi_timer = param;

    if (p_mi_timer->cb)
    {
        p_mi_timer->cb(p_mi_timer->p);
    }
}

mible_status_t mible_timer_start(void *timer_id, uint32_t timeout_value,
                                 void *p_context)
{
    mi_timer_t *p_mi_timer;

    for (uint8_t i = 0; i < MAX_TIMER_MI; i++)
    {
        p_mi_timer = (mi_timer_t *)timer_id;
        if (&mi_timer[i] == p_mi_timer)
        {
            p_mi_timer->p = p_context;
            evt_timer_set(&p_mi_timer->timer, timeout_value, (evt_timer_mode_t)p_mi_timer->mode, timer_callback, p_mi_timer);
            return MI_SUCCESS;
        }
    }

    return MI_ERR_NOT_FOUND;
}

mible_status_t mible_timer_stop(void *timer_id)
{
    mi_timer_t *p_mi_timer = (mi_timer_t *)timer_id;

    for (uint8_t i = 0; i < MAX_TIMER_MI; i++)
    {
        if (&mi_timer[i] == p_mi_timer)
        {
            evt_timer_del(&p_mi_timer->timer);
            memset(&p_mi_timer->timer, 0, sizeof(evt_timer_t));
            return MI_SUCCESS;
        }
    }

    return MI_ERR_NOT_FOUND;
}

mible_status_t mible_record_create(uint16_t record_id, uint8_t len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    if (record_id + NVDS_TAG_USER_BEGIN > NVDS_TAG_MAX)
    {
        return MI_ERR_INVALID_PARAM;
    }

    return MI_SUCCESS;
}

mible_status_t mible_record_delete(uint16_t record_id)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, record_id = %d\n",
                __LINE__, __func__, record_id + NVDS_TAG_USER_BEGIN);
    if (record_id + NVDS_TAG_USER_BEGIN > NVDS_TAG_MAX)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (record_id == RECORD_DFU_INFO)
    {
        uint16_t rd_len = 16;
        uint8_t rd_buffer[16] = {0};
        nvds_get(record_id + NVDS_TAG_USER_BEGIN, &rd_len, rd_buffer);
        mi_app_len = ((mible_dfu_info_t *)rd_buffer)->recv_bytes;
        mi_app_crc = ((mible_dfu_info_t *)rd_buffer)->crc32;
    }

    if (nvds_del(record_id + NVDS_TAG_USER_BEGIN) == NVDS_OK)
    {
        return MI_SUCCESS;
    }
    else
    {
        MI_LOG_INFO("["__FILE__"][Line: %d]: %s nvds_del failed\n", __LINE__, __func__);
        return MI_ERR_INVALID_PARAM;
    }
}

mible_status_t mible_record_read(uint16_t record_id, uint8_t *p_data,
                                 uint8_t len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, record_id = %d, len = %d\n",
                __LINE__, __func__, record_id + NVDS_TAG_USER_BEGIN, len);
    uint16_t rd_len = len;

    if (record_id + NVDS_TAG_USER_BEGIN > NVDS_TAG_MAX)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (nvds_get(record_id + NVDS_TAG_USER_BEGIN, &rd_len, p_data) == NVDS_OK)
    {
        return MI_SUCCESS;
    }
    else
    {
        MI_LOG_INFO("["__FILE__"][Line: %d]: %s nvds_get failed\n",
                    __LINE__, __func__);
        return MI_ERR_INVALID_PARAM;
    }
}

mible_status_t mible_record_write(uint16_t record_id, const uint8_t *p_data,
                                  uint8_t len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, record_id = %d, len = %d\n",
                __LINE__, __func__, record_id + NVDS_TAG_USER_BEGIN, len);
    if (record_id + NVDS_TAG_USER_BEGIN > NVDS_TAG_MAX)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (record_id == RECORD_DFU_INFO)
    {
        MI_LOG_INFO(" crc32 = 0x%08x recv_bytes = 0x%08x\n", ((mible_dfu_info_t *)p_data)->crc32, ((mible_dfu_info_t *)p_data)->recv_bytes);
    }

    if (nvds_put(record_id + NVDS_TAG_USER_BEGIN, len, p_data) == NVDS_OK)
    {
        return MI_SUCCESS;
    }
    else
    {
        MI_LOG_INFO("["__FILE__"][Line: %d]: %s nvds_put failed\n", __LINE__, __func__);
        return MI_ERR_INVALID_PARAM;
    }
}

mible_status_t mible_rand_num_generator(uint8_t *p_buf, uint8_t len)
{
    co_generate_random(p_buf, len);
    return MI_SUCCESS;
}

mible_status_t mible_aes128_encrypt(const uint8_t *key,
                                    const uint8_t *plaintext, uint8_t plen, uint8_t *ciphertext)
{
    uint8_t aes_key[16];
    memcpy(aes_key, key, 16);

    AES_Init(aes_key);
    AES_Encrypt(plaintext, ciphertext, plen, aes_key);

    return MI_SUCCESS;
}

struct miot_cmd
{
    mible_handler_t handler;
    void *arg;
};

mible_status_t mible_task_post(mible_handler_t handler, void *arg)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    if (handler)
    {
        handler(arg);
    }
    return MI_SUCCESS;
}

void mible_tasks_exec(void)
{
    // NULL
}

static CS_I2C_Type *mi_i2c = NULL;
static iic_config_t iic_config;

mible_status_t mible_iic_init(const iic_config_t *p_config, mible_handler_t handler)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    mible_status_t status = MI_ERR_INVALID_PARAM;
    pinmux_t scl_pinmux = PINMUX_I2C0_SCK_CFG;
    pinmux_t sda_pinmux = PINMUX_I2C0_SDA_CFG;
    CS_I2C_Type *i2c = CS_I2C0;
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER, // 7-bit addressing mode
        .speed = I2C_SPEED_100K,
    };
    do
    {
        memset(&iic_config, 0, sizeof(iic_config_t));

        if (!p_config)
        {
            break;
        }
        if (p_config->freq != IIC_100K && p_config->freq != IIC_400K)
        {
            break;
        }
        if (p_config->freq == IIC_400K)
        {
            cfg.speed = I2C_SPEED_400K;
        }
        if (p_config->scl_port != p_config->sda_port)
        {
            break;
        }
        if (p_config->scl_pin >= 32 || p_config->sda_pin >= 32)
        {
            break;
        }
        if (p_config->scl_pin == p_config->sda_pin)
        {
            break;
        }

        DRV_PIN_MUX_SET(p_config->scl_pin, scl_pinmux);
        DRV_PIN_MUX_SET(p_config->sda_pin, sda_pinmux);

        drv_pmu_pin_mode_set((1u << p_config->scl_pin) | (1u << p_config->sda_pin), PMU_PIN_MODE_OD);
        drv_i2c_init(mi_i2c, &cfg);
        memcpy(&iic_config, p_config, sizeof(iic_config_t));
        mi_i2c = i2c;

        status = MI_SUCCESS;
    } while (0);

    return status;
}

void mible_iic_uninit(void)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    if (mi_i2c && mi_i2c == CS_I2C0)
    {
        //        i2c_close_x(mi_i2c);
        mi_i2c = NULL;
        DRV_PIN_MUX_SET(iic_config.scl_pin, PINMUX_DEBUG_MODE_CFG);
        DRV_PIN_MUX_SET(iic_config.sda_pin, PINMUX_DEBUG_MODE_CFG);
        memset(&iic_config, 0, sizeof(iic_config));
    }
}

mible_status_t mible_iic_tx(uint8_t addr, uint8_t *p_out, uint16_t len, bool no_stop)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    bool res = false;

    if (!p_out || !len)
    {
        return MI_ERR_INVALID_PARAM;
    }
    do
    {
        if (mi_i2c != CS_I2C0)
        {
            break;
        }
        res = drv_i2c_master_write(mi_i2c, addr, p_out, len, 0);
    } while (0);

    return res ? MI_SUCCESS : MI_ERR_INTERNAL;
}

mible_status_t mible_iic_rx(uint8_t addr, uint8_t *p_in, uint16_t len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    bool res = false;

    if (!p_in || !len)
    {
        return MI_ERR_INVALID_PARAM;
    }
    do
    {
        if (mi_i2c != CS_I2C0)
            break;

        res = drv_i2c_master_read(mi_i2c, addr, NULL, 0, p_in, len, 0);

    } while (0);

    return res ? MI_SUCCESS : MI_ERR_INTERNAL;
}

int mible_iic_scl_pin_read(uint8_t port, uint8_t pin)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    int pin_value;
    pinmux_t pinmux_func;

    iic_config_t __iic_config;
    memset(&__iic_config, 0, sizeof(__iic_config));

    // i2c initialize unsuccesfully
    if (0 == memcmp(&__iic_config, &iic_config, sizeof(iic_config)))
    {
        return MI_ERR_INVALID_PARAM;
    }
    if (pin != iic_config.scl_pin && pin != iic_config.sda_pin)
    {
        return MI_ERR_INVALID_PARAM;
    }
    if (port != iic_config.scl_port && port != iic_config.sda_port)
    {
        return MI_ERR_INVALID_PARAM;
    }
    drv_pmu_pin_mode_set((1u << pin), PMU_PIN_MODE_PU);
    DRV_PIN_MUX_SET(pin, PINMUX_GPIO_MODE_CFG);
    drv_gpio_set_dir(CS_GPIO0, (1u << pin), GPIO_DIR_INPUT);
    pin_value = drv_gpio_read(CS_GPIO0, (1u << pin)) >> pin;

    if (1 == port)
    {
        // do nothing
    }
    else
    {
        if (pin == iic_config.scl_pin)
        {
            pinmux_func = PINMUX_I2C0_SCK_CFG;
        }

        else
        {
            pinmux_func = PINMUX_I2C0_SDA_CFG;
        }
    }

    drv_pmu_pin_mode_set((1u << pin), PMU_PIN_MODE_OD);
    DRV_PIN_MUX_SET(pin, pinmux_func);

    return pin_value;
}

mible_status_t mible_gatts_rw_auth_reply(uint16_t conn_handle,
                                         uint8_t status, uint16_t char_value_handle, uint8_t offset,
                                         uint8_t *p_value, uint8_t len, uint8_t type)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    return MI_SUCCESS;
}

mible_status_t mible_gatts_service_init(mible_gatts_db_t *p_server_db)
{
    int i, j;
    uint8_t char_uuid[2] = {0};
    uint8_t serv_uuid128[16] = {0};
    uint8_t nb_att = 0;
    uint8_t att_num = 0;
    uint8_t uuid_len = CS_UUID_16BIT;
    uint8_t *p_serv_uuid = NULL;
    uint16_t start_hdl = 0;

    cs_event_callback_reg(gatt_event_cb);
    cs_gatt_serv_t *p_serv = (cs_gatt_serv_t *)mible_malloc(sizeof(cs_gatt_serv_t) * p_server_db->srv_num);
    CS_ASSERT(p_serv != NULL);
    memset(p_serv, 0, sizeof(cs_gatt_serv_t) * p_server_db->srv_num);
    g_serv = p_serv;

    for (i = 0; i < p_server_db->srv_num; i++)
    {
        mible_gatts_srv_db_t *p_srv_db = &p_server_db->p_srv_db[i];
        if (p_srv_db->srv_type != MIBLE_PRIMARY_SERVICE)
        {
            return MI_ERR_INVALID_PARAM;
        }
        if (p_srv_db->srv_uuid.type)
        {
            uuid_len = CS_UUID_128BIT;
        }
        p_serv->uuid = (uint8_t *)mible_malloc(uuid_len);
        CS_ASSERT(p_serv->uuid != NULL);
        p_serv->uuid_len = uuid_len;
        memcpy((void *)p_serv->uuid, p_srv_db->srv_uuid.uuid128, uuid_len);

        nb_att++;
        cs_gatt_item_t *p_char = (cs_gatt_item_t *)mible_malloc(sizeof(cs_gatt_item_t) * p_srv_db->char_num * 3);
        CS_ASSERT(p_char != NULL);
        memset((void *)p_char, 0, sizeof(cs_gatt_item_t) * p_srv_db->char_num * 3);
        cs_gatt_item_t *p_char_og = p_char;

        for (j = 0; j < p_srv_db->char_num; j++)
        {
            mible_gatts_char_db_t *p_char_db = &p_srv_db->p_char_db[j];
            // characteristic define
            p_char->uuid = (uint8_t *)mible_malloc(CS_UUID_16BIT);
            CS_ASSERT(p_char->uuid != NULL);
            p_char->att_prop = CS_ATT_PROP_READ;
            p_char->uuid_len = CS_UUID_16BIT;
            memcpy((void *)p_char->uuid, ob_att_char_def, CS_UUID_16BIT);
            nb_att++;
            p_char++;
            att_num++;
            // characteristic value
            p_char_db->char_value_handle = nb_att;
            if (p_char_db->char_uuid.type)
            {
                p_char->uuid = (uint8_t *)mible_malloc(CS_UUID_128BIT);
                CS_ASSERT(p_char->uuid != NULL);
                p_char->uuid_len = CS_UUID_128BIT;
                memcpy((void *)p_char->uuid, p_char_db->char_uuid.uuid128, CS_UUID_128BIT);
            }
            else
            {
                p_char->uuid = (uint8_t *)mible_malloc(CS_UUID_16BIT);
                CS_ASSERT(p_char->uuid != NULL);
                char_uuid[0] = p_char_db->char_uuid.uuid16 & 0xFF;
                char_uuid[1] = (p_char_db->char_uuid.uuid16 >> 8) & 0xFF;
                p_char->uuid_len = CS_UUID_16BIT;
                memcpy((void *)p_char->uuid, char_uuid, CS_UUID_16BIT);
            }
            p_char->att_prop = p_char_db->char_property;
            if (p_char->att_prop & MIBLE_READ)
            {
                p_char->att_perm_read = false;
            }
            if (p_char->att_prop & MIBLE_WRITE || p_char->att_prop & MIBLE_WRITE_WITHOUT_RESP)
            {
                p_char->att_perm_write = false;
            }
            nb_att++;
            p_char++;
            att_num++;
            // descriptor value
            if ((p_char_db->char_property & MIBLE_INDICATE) || (p_char_db->char_property & MIBLE_NOTIFY))
            {
                p_char->uuid = (uint8_t *)mible_malloc(CS_UUID_16BIT);
                CS_ASSERT(p_char->uuid != NULL);
                uint8_t desc_uuid[2] = {0x02, 0x29};
                memcpy((void *)p_char->uuid, desc_uuid, CS_UUID_16BIT);
                p_char->uuid_len = CS_UUID_16BIT;
                p_char->att_prop = CS_ATT_PROP_READ | CS_ATT_PROP_WRITE;
                p_char->att_perm_read = false;
                p_char->att_perm_write = false;
                nb_att++;
                p_char++;
                att_num++;
            }
        }

        p_serv->att_num = att_num;
        //        p_serv->uuid = p_serv_uuid;
        p_serv->item = p_char_og;
        cs_gatts_add_service(p_serv, &start_hdl);

        p_srv_db->srv_handle = start_hdl;
        for (j = 0; j < p_srv_db->char_num; j++)
        {
            mible_gatts_char_db_t *p_char_db = &p_srv_db->p_char_db[j];
            // Set handle
            p_char_db->char_value_handle += start_hdl;
            // Set att auth list
            if (p_char_db->rd_author || p_char_db->wr_author)
            {
                att_auth_list_add(p_char_db->char_value_handle, p_char_db->rd_author, p_char_db->wr_author);
            }
            // Set att vlaue list
            if (p_char_db->p_value)
            {
                att_val_list_add(p_char_db->char_value_handle, p_char_db->p_value, p_char_db->char_value_len);
            }
            if ((p_char_db->char_property & MIBLE_INDICATE) || (p_char_db->char_property & MIBLE_NOTIFY))
            {
                uint8_t desc_value[2] = {0x01, 0x00};
                att_val_list_add(p_char_db->char_value_handle + 1, desc_value, 2);
            }
        }
        att_num = 0;
        p_serv++;
    }

    mible_arch_evt_param_t param;
    memset(&param, 0, sizeof(param));
    param.srv_init_cmp.status = MI_SUCCESS;
    param.srv_init_cmp.p_gatts_db = p_server_db;
    mible_arch_event_callback(MIBLE_ARCH_EVT_GATTS_SRV_INIT_CMP, &param);

    return MI_SUCCESS;
}

void mible_gatts_service_deinit()
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    while (g_serv != NULL && (g_serv->uuid_len == 2 || g_serv->uuid_len == 16))
    {
        for (int i = 0; i < g_serv->att_num; i++)
        {
            if (g_serv->item != NULL)
            {
                if (g_serv->item->uuid != NULL)
                {
                    mible_free((void *)g_serv->item->uuid);
                }
                mible_free((void *)g_serv->item);
            }
            g_serv->item++;
        }
        if (g_serv->uuid != NULL)
        {
            mible_free((void *)g_serv->uuid);
        }
        mible_free(g_serv);
        g_serv++;
    }
}

mible_status_t mible_gatts_value_set(uint16_t srv_handle,
                                     uint16_t value_handle, uint8_t offset, uint8_t *p_value, uint8_t len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, value_handle = %d, len = %d\n",
                __LINE__, __func__, value_handle, len);
    if (p_value == NULL || len == 0)
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return att_val_list_add(value_handle, p_value, len);
}

mible_status_t mible_gatts_value_get(uint16_t srv_handle,
                                     uint16_t value_handle, uint8_t *p_value, uint8_t *p_len)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, value_handle = %d\n",
                __LINE__, __func__, value_handle);
    return att_value_list_get(value_handle, p_value, p_len);
}

mible_status_t mible_gatts_notify_or_indicate(uint16_t conn_handle,
                                              uint16_t srv_handle, uint16_t char_value_handle, uint8_t offset,
                                              uint8_t *p_value, uint8_t len, uint8_t type)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s, gatt notify, value_handle: %d, type:%d\n",
                __LINE__, __func__, char_value_handle, type);
    cs_gatts_hvx_t hvx;

    if (p_value == NULL || len == 0)
    {
        return MI_ERR_INVALID_LENGTH;
    }

    if (type == 1)
    {
        hvx.type = CS_HANDLE_VALUE_NTF;
    }
    else
    {
        hvx.type = CS_HANDLE_VALUE_IND;
    }
    hvx.att_hdl = char_value_handle;
    hvx.data = p_value;
    hvx.len = len;
    uint32_t ret = cs_gatts_send_hvx(conn_handle, &hvx);

    return MI_SUCCESS;
}

static uint32_t cs_flash_capacity = 0;
// OTA
mible_status_t mible_nvm_init(void)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    uint32_t flash_id = 0;

    drv_sf_enable(CS_SF, 0);
    flash_id = drv_sf_read_id(CS_SF, 0);
    if (flash_id == 0xFFFFFF || flash_id == 0x000000)
    {
        return MI_ERR_INTERNAL;
    }
    cs_flash_capacity = 1u << (flash_id & 0xFF);

    return MI_SUCCESS;
}

/**
 * @brief   Function for reading data from Non-Volatile Memory.
 * @param   [out] p_data:  Pointer to data to be restored.
 *          [in] length:   Data size in bytes.
 *          [in] address:  Address in Non-Volatile Memory to read.
 * @return  MI_ERR_INTERNAL:  invalid NVM address.
 *          MI_SUCCESS
 * */
mible_status_t mible_nvm_read(void *p_data, uint32_t length, uint32_t address)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    uint8_t buf[256] = {0};
    uint32_t op_size = 0;
    uint32_t actual_size = 0;
    uint8_t *p = (uint8_t *)p_data;

    if (0 == length || !p_data)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (address + length > cs_flash_capacity)
    {
        return MI_ERR_NO_MEM;
    }

    drv_sf_enable(CS_SF, 0);

    do
    {
        op_size = (length > sizeof(buf)) ? sizeof(buf) : length;
        actual_size = op_size;
        if (op_size % 16)
        {
            op_size = (op_size + 0xF) & ~0xF;
        }

        drv_sf_read(CS_SF, 0, address, buf, op_size);
        memcpy(p, buf, actual_size);
        address += actual_size;
        p += actual_size;
        length -= actual_size;
    } while (length != 0);

    return MI_SUCCESS;
}

/**
 * @brief   Writes data to Non-Volatile Memory.
 * @param   [in] p_data:   Pointer to data to be stored.
 *          [in] length:   Data size in bytes.
 *          [in] address:  Start address used to store data.
 * @return  MI_ERR_INTERNAL:  invalid NVM address.
 *          MI_SUCCESS
 * */
mible_status_t mible_nvm_write(void *p_data, uint32_t length, uint32_t address)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    if (0 == length || !p_data)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (address + length > cs_flash_capacity - 4096)
    {
        return MI_ERR_NO_MEM;
    }

    drv_sf_enable(CS_SF, 0);

    if (!(address & 0xFFF))
    {
        drv_sf_erase_sector(CS_SF, 0, address);
    }

    if (((address + length) & ~0xFFF) > (address & ~0xFFF))
    {
        uint32_t _address_end = (address + length) & ~0xFFF;
        uint32_t _address_start = (address + 0xFFF) & ~0xFFF;
        do
        {
            drv_sf_erase_sector(CS_SF, 0, _address_start);
            _address_start += 0x1000;
        } while (_address_start <= _address_end);
    }

    drv_sf_write(CS_SF, 0, address, p_data, length);

    return MI_SUCCESS;
}

mible_status_t mible_upgrade_firmware(void)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    mible_status_t err;
    uint8_t buf1[256] = {0};
    uint8_t buf2[256] = {0};
    uint8_t retry_cnt = 0;

    uint32_t mbr_addr_rd, mbr_length_rd, mbr_length_cal;
    uint16_t mbr_crc16_cal, mbr_crc16_rd;
    uint32_t addr, length, op_length;

    drv_sf_enable(CS_SF, 0);

    length = mi_app_len;
    mbr_length_cal = mi_app_len;

    addr = DFU_NVM_START;
    mbr_crc16_cal = 0;
    uint32_t mi_crc = 0;
    do
    {
        op_length = length > sizeof(buf1) ? sizeof(buf1) : length;

        for (retry_cnt = 0; retry_cnt < 2; retry_cnt++)
        {
            err = mible_nvm_read(buf1, op_length, addr);
            if (MI_SUCCESS != err)
            {
                continue;
            }
            err = mible_nvm_read(buf2, op_length, addr);
            if (MI_SUCCESS != err)
            {
                continue;
            }

            if (!memcmp(buf1, buf2, op_length))
            {
                break;
            }
        }

        if (retry_cnt >= 2)
        {
            return MI_ERR_INTERNAL;
        }

        mi_crc = soft_crc32(buf1, op_length, mi_crc);
        mbr_crc16_cal = cs_crc16_ccitt(mbr_crc16_cal, buf1, op_length);

        addr += op_length;
        length -= op_length;
    } while (length != 0);

    if (length != 0 || mi_crc != mi_app_crc)
        return MI_ERR_INTERNAL;

    CS_CRITICAL_BEGIN();
    mbr_write_part(PART_TYPE_APP, DFU_NVM_START, mbr_length_cal, mbr_crc16_cal);
    mbr_read_part(PART_TYPE_APP, &mbr_addr_rd, &mbr_length_rd, &mbr_crc16_rd);
    CS_CRITICAL_END();

    if (mbr_addr_rd != DFU_NVM_START || mbr_length_rd != mbr_length_cal || mbr_crc16_cal != mbr_crc16_rd)
    {
        return MI_ERR_INTERNAL;
    }

    drv_pmu_force_reboot();

    return MI_ERR_BUSY;
}

void *mible_malloc(size_t size)
{
    int *p = cs_ble_mem_malloc(size, CS_MEM_HOST);

    if (!p)
    {
        return NULL;
    }
    return (void *)p;
}

void mible_free(void *ptr)
{
    cs_ble_mem_free(ptr);
}

void cs_mible_gatts_service_restore(void)
{
    MI_LOG_INFO("["__FILE__"][Line: %d]: %s\n", __LINE__, __func__);
    att_auth_list_clear();
    att_value_list_clear();
    cs_event_callback_unreg(gatt_event_cb);
    mible_gatts_service_deinit();
}