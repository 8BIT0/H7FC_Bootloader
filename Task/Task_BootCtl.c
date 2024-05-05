#include "Task_BootCtl.h"
#include "Srv_OsCommon.h"
#include "Srv_Upgrade.h"
#include "Storage.h"
#include "Bsp_Uart.h"
#include "Bsp_USB.h"

#define VCP_ATTACH_TIMEOUT  100 /* 100ms */
#define BOOTCTL_MIN_PERIOD  50  /* 50ms 20Hz */
#define JUMP_WINDOW_TIME    500 /* default window time */

#define RunningStage On_Boot

typedef enum
{
    None_Update = 0,
    Boot_Update,
    App_Update,
    Module_Update,
} Boot_UpdateType_List;

typedef enum
{
    BootPort_None = 0,
    BootPort_USB,
    BootPort_UART,
} BootPort_Type_List;

typedef struct
{
    BootPort_Type_List type;
    uint32_t addr;

} BootPortProtoObj_TypeDef;

typedef struct
{
    uint32_t period;
    uint32_t jump_time;
    Boot_UpdateType_List update_type;

    bool init_state;

    BootPortProtoObj_TypeDef port_obj;
} BootCtlMonitor_TypeDef;

/* internal vriable */
static BootCtlMonitor_TypeDef BootMonitor = {
    .init_state = false,
    .port_obj = {
        .addr = 0,
        .type = BootPort_None,
    },
};

void TaskBootCtl_Init(uint32_t period)
{
    SrvUpgrade.init(RunningStage, 500);

    /* port init */
    BspUSB_VCP.init(&BootMonitor.port_obj);

    /* get base info from storage module */
    BootMonitor.period = period;
    if (period < BOOTCTL_MIN_PERIOD)
        BootMonitor.period = BOOTCTL_MIN_PERIOD;
}

void TaskBootCtl_Core(const void *argument)
{
    uint32_t pre_time = SrvOsCommon.get_os_ms();
    uint32_t sys_time = 0;
    BootMonitor.jump_time = pre_time + JUMP_WINDOW_TIME;

    while(1)
    {
        sys_time = SrvOsCommon.get_os_ms();
        
        if (sys_time >= BootMonitor.jump_time)
        {
            /* jump to app */
        }

        SrvOsCommon.precise_delay(&pre_time, BootMonitor.period);
    }
}

static void TaskBootCtl_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp)
{
    BootPortProtoObj_TypeDef *p_Obj = NULL;

    if (Obj_addr)
    {
        p_Obj = (BootPortProtoObj_TypeDef *)Obj_addr;

        if (p_Obj->addr && (p_Obj->type == BootPort_USB))
            *time_stamp = SrvOsCommon.get_os_ms();
    }
}

static void TaskBootCtl_Send(uint8_t *p_buf, uint16_t len)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    if (p_buf && len)
    {
        /* send through VCP */
        if (BspUSB_VCP.check_connect(sys_time, VCP_ATTACH_TIMEOUT))
        {
            BspUSB_VCP.send(p_buf, len);

            /* wait semaphore */
        }

        /* send through default uart port */

    }
}
