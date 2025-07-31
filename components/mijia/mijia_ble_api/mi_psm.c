// #ifdef S110
// #include "pstorage.h"
// #include "mible_log.h"
// #include "mible_type.h"

// #define BLK_SIZE      32
// #define BLK_NUM       4
// #define REC_TABLE_ID  0

// #ifndef FDS_SUCCESS
// #define FDS_SUCCESS                    NRF_SUCCESS
// #endif

// static uint8_t m_flags_pstorage_is_busy;
// static pstorage_handle_t m_storage_handle;


// static void pstorage_cb_handler(pstorage_handle_t * p_handle,
//                                    uint8_t             op_code,
//                                    uint32_t            result,
//                                    uint8_t           * p_data,
//                                    uint32_t            data_len)
// {
//     MI_LOG_INFO("pstorage block %X, opcode %d, result %d\n", p_handle, op_code, result);
//     m_flags_pstorage_is_busy = 0;
// }

// static __inline void pstorage_blocking_begin()
// {
//     m_flags_pstorage_is_busy = 1;
// }

// static __inline void pstorage_blocking_end()
// {
//     while(m_flags_pstorage_is_busy);
// }


// static uint16_t recordid_2_blocknum(uint16_t record_id)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;
//     uint32_t          rec_table[BLK_NUM-1];
//     err_code = pstorage_block_identifier_get(&m_storage_handle, REC_TABLE_ID, &block_handle);
//     MI_ERR_CHECK(err_code);

//     err_code = pstorage_load((uint8_t*)rec_table, &block_handle, sizeof(rec_table), 0);
//     if (err_code == NRF_SUCCESS) {
//         MI_LOG_DEBUG("CHK REC%d IN TABLE\n", record_id);
//         MI_LOG_HEXDUMP(rec_table, sizeof(rec_table));
//         for (uint32_t i = 0; i < BLK_NUM-1; i++) {
//             if (record_id == rec_table[i])
//                 return i+1;
//         }
//     }

//     return 0;
// }

// static uint16_t recordid_del(uint16_t record_id)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;
//     uint32_t          rec_table[BLK_NUM-1];
//     err_code = pstorage_block_identifier_get(&m_storage_handle, REC_TABLE_ID, &block_handle);
//     MI_ERR_CHECK(err_code);

//     err_code = pstorage_load((uint8_t*)rec_table, &block_handle, sizeof(rec_table), 0);
//     if (err_code == NRF_SUCCESS) {
//         MI_LOG_DEBUG("DEL REC%d IN TABLE\n", record_id);
//         MI_LOG_HEXDUMP(rec_table, sizeof(rec_table));
//         for (uint32_t i = 0; i < BLK_NUM-1; i++) {
//             if (rec_table[i] == record_id) {
//                 rec_table[i] = 0;
//                 pstorage_blocking_begin();
//                 pstorage_update(&block_handle, (uint8_t*)&rec_table, sizeof(rec_table), 0);
//                 pstorage_blocking_end();
//                 return 0;
//             }
//         }
//     }

//     return 0;
// }

// static uint16_t recordid_add(uint16_t record_id)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;
//     uint32_t          rec_table[BLK_NUM-1];
//     err_code = pstorage_block_identifier_get(&m_storage_handle, REC_TABLE_ID, &block_handle);
//     MI_ERR_CHECK(err_code);

//     err_code = pstorage_load((uint8_t*)rec_table, &block_handle, sizeof(rec_table), 0);
//     if (err_code == NRF_SUCCESS) {
//         MI_LOG_DEBUG("DEL REC%d IN TABLE\n", record_id);
//         MI_LOG_HEXDUMP(rec_table, sizeof(rec_table));
//         for (uint32_t i = 0; i < BLK_NUM-1; i++) {
//             if (rec_table[i] == 0) {
//                 rec_table[i] = record_id;
//                 pstorage_blocking_begin();
//                 pstorage_update(&block_handle, (uint8_t*)&rec_table, sizeof(rec_table), 0);
//                 pstorage_blocking_end();
//                 return 0;
//             }
//         }
//     }

//     return 0;
// }

// void mi_psm_init(void)
// {
//     uint32_t err_code;
//     pstorage_module_param_t param;
//     param.block_size  = BLK_SIZE;
//     param.block_count = BLK_NUM;
//     param.cb          = pstorage_cb_handler;

//     err_code = pstorage_register(&param, &m_storage_handle);
//     MI_ERR_CHECK(err_code);
// }

// int mi_psm_record_write(uint16_t record_id, const uint8_t *p_data, uint16_t len)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;
//     bool              is_newone;

//     uint16_t blk_num = recordid_2_blocknum(record_id);
//     if (blk_num == 0) {
//         MI_LOG_WARNING("ADD NEW REC %d\n", record_id);
//         is_newone = true;
//         blk_num = recordid_add(record_id);
//     } else {
//         is_newone = false;
//     }

//     err_code = pstorage_block_identifier_get(&m_storage_handle, blk_num, &block_handle);

//     if (err_code == NRF_SUCCESS)
//     {
//         MI_LOG_INFO("WR REC %d\n", record_id);
//         pstorage_blocking_begin();
//         if (is_newone)
//             pstorage_store(&block_handle, (uint8_t*)p_data, len, 0);
//         else
//             pstorage_update(&block_handle, (uint8_t*)p_data, len, 0);
//         pstorage_blocking_end();
//     }
//     else
//     {
//         MI_LOG_INFO("WR REC %d FAILED\n", record_id);
//         //Request update record entry table block.
//         recordid_del(record_id);
//     }
//     return (err_code);
// }

// /**@brief Flash Read function type. */
// int mi_psm_record_read(uint16_t record_id, uint8_t *p_data, uint16_t len)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;

//     uint16_t blk_num = recordid_2_blocknum(record_id);
//     if (blk_num == 0) {
//         MI_LOG_ERROR("REC %d not found\n", record_id);
//         return MI_ERR_INVALID_PARAM;
//     }

//     err_code = pstorage_block_identifier_get(&m_storage_handle, blk_num, &block_handle);

//     if (err_code == NRF_SUCCESS)
//     {
//         MI_LOG_INFO("RD REC %d\n", record_id);
//         //Request clearing of the block.
//         err_code = pstorage_load(p_data, &block_handle, len, 0);
//     }
//     else
//     {
//         MI_LOG_INFO("RD REC %d FAILED\n", record_id);
//         //Request update record entry table block.
//         recordid_del(record_id);
//     }

//     return (err_code);
// }

// int mi_psm_record_delete(uint16_t record_id)
// {
//     uint32_t          err_code;
//     pstorage_handle_t block_handle;

//     uint16_t blk_num = recordid_2_blocknum(record_id);
//     if (blk_num == 0) {
//         MI_LOG_ERROR("REC %d not found\n", record_id);
//         return MI_ERR_INVALID_PARAM;
//     }

//     err_code = pstorage_block_identifier_get(&m_storage_handle, blk_num, &block_handle);

//     if (err_code == NRF_SUCCESS)
//     {
//         MI_LOG_INFO("DEL REC %d\n", record_id);
//         //Request clearing of the block.
//         pstorage_blocking_begin();
//         err_code = pstorage_clear(&block_handle, BLK_SIZE);
//         pstorage_blocking_end();
//     }

//     if (err_code == NRF_SUCCESS)
//     {
//         MI_LOG_INFO("DEL REC ID\n");
//         //Request update record entry table block.
//         recordid_del(record_id);
//     }

//     return err_code;
// }

// #else

// //#include "fds.h"
// //#include "nrf_log.h"
// //#include "nrf_log_ctrl.h"
// #include "mi_psm.h"
// #include "mible_type.h"

// #define MI_RECORD_FILE_ID              0x4D49       // file used to storage
// #define MI_RECORD_KEY                  0xBEEF


// #define FDS_SUCCESS                    0


// static volatile uint8_t m_psm_init_done = 0;
// extern void mible_arch_event_callback(mible_arch_event_t evt,
//         mible_arch_evt_param_t* param);

// typedef enum
// {
//     FDS_EVT_INIT,       //!< Event for @ref fds_init.
//     FDS_EVT_WRITE,      //!< Event for @ref fds_record_write and @ref fds_record_write_reserved.
//     FDS_EVT_UPDATE,     //!< Event for @ref fds_record_update.
//     FDS_EVT_DEL_RECORD, //!< Event for @ref fds_record_delete.
//     FDS_EVT_DEL_FILE,   //!< Event for @ref fds_file_delete.
//     FDS_EVT_GC          //!< Event for @ref fds_gc.
// } fds_evt_id_t;

// typedef uint32_t ret_code_t;

// typedef struct
// {
//     fds_evt_id_t id;        //!< The event ID. See @ref fds_evt_id_t.
//     ret_code_t   result;    //!< The result of the operation related to this event.
//     union
//     {
//         struct
//         {
//             uint32_t record_id;
//             uint16_t file_id;
//             uint16_t record_key;
//             bool     is_record_updated;
//         } write; //!< Information for @ref FDS_EVT_WRITE and @ref FDS_EVT_UPDATE events.
//         struct
//         {
//             uint32_t record_id;
//             uint16_t file_id;
//             uint16_t record_key;
//         } del; //!< Information for @ref FDS_EVT_DEL_RECORD and @ref FDS_EVT_DEL_FILE events.
//     };
// } fds_evt_t;



// static void mi_psm_fds_evt_handler(fds_evt_t const * const p_fds_evt)
// {
//     mible_arch_evt_param_t param;
//     mible_arch_event_t event;
//     switch (p_fds_evt->id) {
//     case FDS_EVT_INIT:
//         if (p_fds_evt->result == FDS_SUCCESS) {
//            // NRF_LOG_INFO("MI PSM INIT SUCCESS\n");
//             m_psm_init_done = 1;
//         }else{
//            // // NRF_LOG_ERROR("MI PSM INIT FAILED\n");
//         }
//         break;

//     case FDS_EVT_WRITE:
//         // NRF_LOG_INFO("REC %X \n", p_fds_evt->write.record_key);
//         event = MIBLE_ARCH_EVT_RECORD_WRITE;
//         if ((uint32_t)p_fds_evt->write.file_id == MI_RECORD_FILE_ID) {
//             param.record.id = (uint16_t)p_fds_evt->write.record_key;
//             if (p_fds_evt->result == FDS_SUCCESS) {
//                 // NRF_LOG_INFO("WRITE SUCCESS\n");
//                 param.record.status = MI_SUCCESS;
//             } else {
//                 // NRF_LOG_ERROR("WRITE FAILED\n");
//                 param.record.status = MIBLE_ERR_UNKNOWN;
//             }
//             mible_arch_event_callback(event, &param);
//         }
//         break;

//     case FDS_EVT_UPDATE:
//         // NRF_LOG_INFO("REC %X \n", p_fds_evt->write.record_key);
//         event = MIBLE_ARCH_EVT_RECORD_WRITE;
//         if ((uint32_t)p_fds_evt->write.file_id == MI_RECORD_FILE_ID) {
//             param.record.id = (uint16_t)p_fds_evt->write.record_key;
//             if (p_fds_evt->result == FDS_SUCCESS) {
//                 // NRF_LOG_INFO("REWRITE SUCCESS\n");
//                 param.record.status = MI_SUCCESS;
//             } else {
//                 // NRF_LOG_ERROR("REWRITE FAILED\n");
//                 param.record.status = MIBLE_ERR_UNKNOWN;
//             }
//             mible_arch_event_callback(event, &param);
//         }
//         break;

//     case FDS_EVT_DEL_RECORD:
//         // NRF_LOG_INFO("REC %X \n", p_fds_evt->del.record_key);
//         event = MIBLE_ARCH_EVT_RECORD_DELETE;
//         if ((uint32_t)p_fds_evt->del.file_id == MI_RECORD_FILE_ID) {
//             param.record.id = (uint16_t)p_fds_evt->del.record_key;
//             if (p_fds_evt->result == FDS_SUCCESS) {
//                 // NRF_LOG_INFO("DELETE SUCCESS\n");
//                 param.record.status = MI_SUCCESS;
//             } else {
//                 // NRF_LOG_ERROR("DELETE FAILED\n");
//                 param.record.status = MIBLE_ERR_UNKNOWN;
//             }
//             mible_arch_event_callback(event, &param);
//         }
//         break;

//     case FDS_EVT_DEL_FILE:
//         if (p_fds_evt->result == FDS_SUCCESS) {
//             if ((uint32_t)p_fds_evt->del.file_id == MI_RECORD_FILE_ID) {
//                 // NRF_LOG_INFO("FDS_EVT_DEL_FILE SUCCESS\n");
//             }
//         }
//         break;

//     case FDS_EVT_GC:
//         if (p_fds_evt->result == FDS_SUCCESS) {
//             // NRF_LOG_INFO("FDS_EVT_GC SUCCESS\n");
//         }else{
//             // NRF_LOG_INFO("FDS_EVT_GC FAILED\n");
//         }
//         break;
//     }

// #ifdef DEBUG
//     fds_stat_t stat;
//     fds_stat(&stat);
//     NRF_LOG_WARNING("FDS STAT:\n");
//     NRF_LOG_RAW_INFO(" used:\t %d\n free:\t %d\n gc:\t %d\n", stat.words_used<<2, stat.largest_contig<<2, stat.freeable_words<<2);
// #endif
// }

// void mi_psm_init(void)
// {
//     uint32_t errno;
//     errno = fds_register(mi_psm_fds_evt_handler);
// //    APP_ERROR_CHECK(errno);
// //
//     errno = fds_init();
// //    APP_ERROR_CHECK(errno);

//     // Block here until psm init finished.
//     while(!m_psm_init_done);

// }

// /**@brief Flash Write function type. */
// mible_status_t mi_psm_record_write(uint16_t rec_key, const uint8_t *in, uint16_t in_len)
// {
// //    uint32_t ret = 0;
// //    fds_record_t        record;
// //    fds_record_desc_t   record_desc;
// //    fds_find_token_t    ftok = {0};

// //#if (NRF_SD_BLE_API_VERSION<=3)
// //    // Set up data.
// //    fds_record_chunk_t  record_chunk;
// //    record_chunk.p_data         = in;
// //    record_chunk.length_words   = CEIL_DIV(in_len, sizeof(uint32_t));
// //    // Set up record.

// //    record.data.p_chunks     = &record_chunk;
// //    record.data.num_chunks   = 1;
// //#else

// //    if (in_len % 4) {
// //        // NRF_LOG_ERROR("mi psm invalid length\n");
// //        return MI_ERR_INVALID_LENGTH;
// //    }

// //    record.data.p_data       = in;
// //    record.data.length_words = in_len/4;
// //#endif
// //    record.file_id           = MI_RECORD_FILE_ID;
// //    record.key               = rec_key;

// //    ret = fds_record_find(MI_RECORD_FILE_ID, rec_key, &record_desc, &ftok);
// //    if (ret == FDS_SUCCESS)
// //    {
// //        ret = fds_record_update(&record_desc, &record);
// //    }
// //    else {
// //        ret = fds_record_write(&record_desc, &record);
// //    }

// //    if (ret == FDS_ERR_NO_SPACE_IN_QUEUES)
// //    {
// //        // NRF_LOG_ERROR("mi psm write KEY %X failed :%d \n", rec_key, ret);
// //        ret = MI_ERR_RESOURCES;
// //    }
// //    else if (ret == FDS_ERR_NO_SPACE_IN_FLASH)
// //    {
// //        // NRF_LOG_ERROR("mi psm startup fds_gc().\n");
// //        ret = fds_gc();
// //        if (ret != FDS_SUCCESS)
// //            // NRF_LOG_ERROR("WTF? \n");
// //        ret = MI_ERR_BUSY;
// //    }
// //    else if (ret != NRF_SUCCESS) {
// //        ret = MIBLE_ERR_UNKNOWN;
// //    }

//     return (mible_status_t)0;
// }

// /**@brief Flash Read function type. */
// mible_status_t mi_psm_record_read(uint16_t rec_key, uint8_t *out, uint16_t out_len)
// {
// //    uint32_t ret = 0;
// //    fds_flash_record_t  flash_record;
// //    fds_record_desc_t   record_desc;
// //    fds_find_token_t    ftok = {0};

// //    ret = fds_record_find(MI_RECORD_FILE_ID, rec_key, &record_desc, &ftok);
// //    if (ret == FDS_SUCCESS)
// //    {
// //        if (fds_record_open(&record_desc, &flash_record) != FDS_SUCCESS)
// //        {
// //            // NRF_LOG_ERROR("mi psm cann't find KEY %X! \n", rec_key);
// //            return MI_ERR_INVALID_PARAM;
// //        }
// //#if (NRF_SD_BLE_API_VERSION<=3)
// //        uint16_t record_len = flash_record.p_header->tl.length_words * sizeof(uint32_t);
// //#else
// //        uint16_t record_len = flash_record.p_header->length_words * sizeof(uint32_t);
// //#endif
// //        if (out_len <= record_len)
// //            memcpy(out, (uint8_t*)flash_record.p_data, out_len);
// //        else
// //            memcpy(out, (uint8_t*)flash_record.p_data, record_len);

// //        if (fds_record_close(&record_desc) != FDS_SUCCESS)
// //        {
// //            // NRF_LOG_ERROR("mi psm close file failed! \n");
// //            ret = MI_ERR_INTERNAL;
// //        }
// //    }
// //    else if (rec_key == 0x10) {
// //        ret = mi_psm_record_read(MI_RECORD_KEY, out, out_len);
// //        if (ret == MI_SUCCESS) {
// //            ret = mi_psm_record_write(rec_key, out, out_len);
// //            if (ret == MI_SUCCESS)
// //                mi_psm_record_delete(MI_RECORD_KEY);
// //            else
// //                return MI_ERR_INTERNAL;
// //        } else {
// //            ret = MI_ERR_INVALID_PARAM;
// //        }
// //    }
// //    else {
// //        // NRF_LOG_ERROR("mi psm cann't read the record %X. \n", rec_key);
// //        ret = MI_ERR_INVALID_PARAM;
// //    }

//     return (mible_status_t)0;
// }

// mible_status_t mi_psm_record_delete(uint16_t rec_key)
// {
// //    uint32_t ret = 0;
// //    fds_record_desc_t   record_desc;
// //    fds_find_token_t    ftok = {0};

// //    ret = fds_record_find(MI_RECORD_FILE_ID, rec_key, &record_desc, &ftok);
// //    if (ret == FDS_SUCCESS)
// //    {
// //        if (fds_record_delete(&record_desc) != FDS_SUCCESS)
// //        {
// //            // NRF_LOG_ERROR("mi psm delete record failed! \n");
// //            ret = MI_ERR_INTERNAL;
// //        }
// //    }
// //    else {
// //        // NRF_LOG_ERROR("mi psm cann't delete the record 0x%X. \n", rec_key);
// //        ret = MI_ERR_INVALID_PARAM;
// //    }

//     return (mible_status_t)0;
// }

// int mi_psm_reset(void)
// {
//     //return fds_file_delete(MI_RECORD_FILE_ID);
// }
// #endif
