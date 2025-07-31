
#include "sdk_config.h"             // SW configuration

#include <stdint.h>
#include <string.h>
#include "cs_log.h"
#include "cmsis_compiler.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif

#define log_debug CS_LOG_DEBUG

// Other tag should NOT conflict with the following tags.
#define NVDS_TAG_HARDFAULT 0xFD

enum {
    FAULT_HardFault = 0,
    FAULT_MemManage,
    FAULT_BusFault,
    FAULT_UsageFault,
    FAULT_SoftFault,
    FAULT_MAX,
};

void chipsea_fault(uint32_t fault_lr, uint32_t fault_sp, uint32_t fault_type)
{
    const char *fault = NULL;
    uint32_t saved_sp = fault_sp;
    const char *reg_name[] = {"R0 ", "R1 ", "R2 ", "R3 ", "R12", "LR ", "PC ", "PSR"};
    uint8_t tag_buf[15] = {0};

    if (fault_lr & (1UL << 2)) {
        saved_sp = __get_PSP();
    }

    switch (fault_type) {
        case FAULT_HardFault:
            fault = "HardFault";
            break;
        case FAULT_MemManage:
            fault = "MemManage";
            break;
        case FAULT_BusFault:
            fault = "BusFault";
            break;
        case FAULT_UsageFault:
            fault = "UsageFault";
            break;
        case FAULT_SoftFault:
            fault = "SoftFault";
            break;
        default:
            fault = "Invalid Type";
            break;
    }

    log_debug("=============== %s ================\r\n", fault);

    log_debug(" %s: %08x %s: %08x %s: %08x %s: %08x\r\n", reg_name[0], *((uint32_t *)saved_sp),
                                                    reg_name[1], *(((uint32_t *)saved_sp + 1)),
                                                    reg_name[2], *(((uint32_t *)saved_sp + 2)),
                                                    reg_name[3], *(((uint32_t *)saved_sp + 3)));
    log_debug(" %s: %08x %s: %08x %s: %08x %s: %08x\r\n", reg_name[4], *(((uint32_t *)saved_sp + 4)),
                                                    reg_name[5], *(((uint32_t *)saved_sp + 5)),
                                                    reg_name[6], *(((uint32_t *)saved_sp + 6)),
                                                    reg_name[7], *(((uint32_t *)saved_sp + 7)));
    log_debug("==========================================\r\n");

    // save the registers value to the flash.
#if (NVDS_SUPPORT)
    memcpy(tag_buf, "lr:", 3);
    memcpy(tag_buf + 3, (uint8_t *)((uint32_t *)saved_sp + 5), sizeof(uint32_t));
    memcpy(tag_buf + 7, "pc:", 3);
    memcpy(tag_buf + 10, (uint8_t *)((uint32_t *)saved_sp + 6), sizeof(uint32_t));
    nvds_put(NVDS_TAG_HARDFAULT, 15, (uint8_t *)tag_buf);
#endif
    while(1);
}


#ifdef TEST_TRIGGER_HARDFAULT
void trigger_hardfault(void)
{
    uint32_t *ptr = (uint32_t *)0xFFFFFFFF;
    uint32_t value;

    __asm volatile (
        "ldr %0, [%1]\n\t"
        : "=r" (value)
        : "r" (ptr)
        : "memory"
    );
}
#endif

