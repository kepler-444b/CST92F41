#include "rwip_config.h"     // SW configuration


#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>           // Common BT Definitions
#include "arch.h"            // Platform Definitions
#include "gapc.h"            // GAPC Definitions

#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)


int om_minble_gatts_proc_handler(ke_msg_id_t const msgid, void const *param, 
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id);
