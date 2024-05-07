#include "Task_BootCtl.h"
#include "Srv_OsCommon.h"
#include "Srv_Upgrade.h"
#include "Storage.h"
#include "Bsp_Uart.h"
#include "Bsp_USB.h"
#include "Bsp_DMA.h"

#define VCP_ATTACH_TIMEOUT      100 /* 100ms */
#define BOOTCTL_MIN_PERIOD      50  /* 50ms 20Hz */
#define JUMP_WINDOW_TIME        500 /* default window time */

#define RunningStage On_Boot

typedef enum
{
    None_Update = 0,
    Boot_Update,
    App_Update,
    Module_Update,
} Boot_UpdateType_List;

typedef struct
{
    uint32_t period;
    Boot_UpdateType_List update_type;

    bool init_state;

    uint8_t Info[1024];
    uint16_t info_size;
} BootCtlMonitor_TypeDef;

/* internal vriable */
static BootCtlMonitor_TypeDef BootMonitor = {
    .init_state = false,
};

void TaskBootCtl_Init(uint32_t period)
{
    SrvUpgrade.init(RunningStage, JUMP_WINDOW_TIME);

    /* get base info from storage module */
    BootMonitor.period = period;
    if (period < BOOTCTL_MIN_PERIOD)
        BootMonitor.period = BOOTCTL_MIN_PERIOD;
}

void TaskBootCtl_Core(const void *argument)
{
    uint32_t pre_time = SrvOsCommon.get_os_ms();
    uint32_t sys_time = 0;

    while(1)
    {
        sys_time = SrvOsCommon.get_os_ms();
        
        if (SrvUpgrade.polling() == Stage_ReadyToJump)
        {
            
        }

        SrvOsCommon.precise_delay(&pre_time, BootMonitor.period);
    }
}

