#include "Task_BootCtl.h"
#include "Srv_OsCommon.h"
#include "Srv_Actuator.h"
#include "Srv_Upgrade.h"
#include "Storage.h"
#include "DataPipe.h"
#include "Bsp_Uart.h"
#include "Bsp_USB.h"
#include "Bsp_DMA.h"

#define VCP_ATTACH_TIMEOUT      100 /* 100ms */
#define BOOTCTL_MIN_PERIOD      50  /* 50ms 20Hz */
#define JUMP_WINDOW_TIME        500 /* default window time */

#define RunningStage On_Boot

DataPipe_CreateDataObj(SrvUpgrade_State_TypeDef, t_BootState);

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
static bool Boot_PipeUpdate = false;

/* internal function */
static void TaskBootCTL_PipeSendCallback(void *pipe_obj);

void TaskBootCtl_Init(uint32_t period)
{
    SrvActuator_Setting_TypeDef actuator_cfg;
    Storage_ItemSearchOut_TypeDef search_out;

    memset(&search_out, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&actuator_cfg, 0, sizeof(SrvActuator_Setting_TypeDef));

    /* read model type from storage */
    search_out = Storage.search(External_Flash, Para_User, ACTUATOR_STORAGE_SECTION_NAME); 
    if (search_out.item_addr)
    {
        /* parameter matched */
        /* update actuator setting config data from storage */
        if ((search_out.item.len != sizeof(SrvActuator_Setting_TypeDef)) || \
            (Storage.get(External_Flash, Para_User, search_out.item, &actuator_cfg, sizeof(actuator_cfg)) != Storage_Error_None))
            actuator_cfg = SrvActuator.default_param();
    }
    else
        actuator_cfg = SrvActuator.default_param();

    SrvActuator.init(actuator_cfg);
    SrvUpgrade.init(RunningStage, JUMP_WINDOW_TIME);

    memset(&JumpState_BootPipe, 0, sizeof(JumpState_BootPipe));
    JumpState_BootPipe.data_addr = DataPipe_DataObjAddr(t_BootState);
    JumpState_BootPipe.data_size = DataPipe_DataSize(t_BootState);
    JumpState_BootPipe.trans_finish_cb = TaskBootCTL_PipeSendCallback;
    DataPipe_Enable(&JumpState_BootPipe);

    /* get base info from storage module */
    BootMonitor.period = period;
    if (period < BOOTCTL_MIN_PERIOD)
        BootMonitor.period = BOOTCTL_MIN_PERIOD;
}

void TaskBootCtl_Core(const void *argument)
{
    uint32_t pre_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        SrvActuator.lock();

        DataPipe_DataObj(t_BootState).stage = SrvUpgrade.polling();
        DataPipe_SendTo(&JumpState_BootPipe, &JumpState_PortPipe);

        switch ((uint8_t)DataPipe_DataObj(t_BootState).stage)
        {
            case Stage_ReadyToJump:
                if (Boot_PipeUpdate && DataPipe_DataObj(t_BootState).All_Port_Disabled)
                {
                    SrvUpgrade.jump();
                    Boot_PipeUpdate = false;
                }
                break;

            default: break;
        }
        SrvOsCommon.precise_delay(&pre_time, BootMonitor.period);
    }
}

static void TaskBootCTL_PipeSendCallback(void *pipe_obj)
{
    if (pipe_obj == &JumpState_BootPipe)
    {
        Boot_PipeUpdate = true;
    }
}

