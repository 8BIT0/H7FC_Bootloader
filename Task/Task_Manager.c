#include "Task_Manager.h"
#include "Task_BootCtl.h"
#include "Task_Protocol.h"
#include "../FCHW_Config.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "shell_port.h"
#include "Storage.h"

#define TaskControl_Period_Def   5  /* unit: ms period 5ms  200Hz  */
#define TaskBootCtl_Period_Def   10 /* unit: ms period 10ms 100Hz */
#define TaskFrameCTL_Period_Def  5  /* unit: ms period 5ms  200Hz  */

osThreadId TaskBootCtl_Handle = NULL;
osThreadId TaskFrameCTL_Handle = NULL;
osThreadId TaskManager_Handle = NULL;
void Task_Manager_Init(void)
{
    DevLED.init(Led1);
#if defined MATEKH743_V1_5
    DevLED.init(Led2);
    DevLED.init(Led3);

    DebugPin.init(Debug_PC0);
    DebugPin.init(Debug_PC1);
    DebugPin.init(Debug_PC2);
    DebugPin.init(Debug_PC3);
    DebugPin.init(Debug_PB3);
    DebugPin.init(Debug_PB4);
    DebugPin.init(Debug_PB5);
    DebugPin.init(Debug_PB6);
    DebugPin.init(Debug_PB10);
#endif
    /* vol ADC init */

    /* cur ADC init */

    osThreadDef(ManagerTask, Task_Manager_CreateTask, osPriorityLow, 0, 1024);
    TaskManager_Handle = osThreadCreate(osThread(ManagerTask), NULL);

    osKernelStart();
}

void Task_Manager_CreateTask(void)
{
    bool init = false;
    Storage_ModuleState_TypeDef storage_module_enable;
    Storage_ExtFLashDevObj_TypeDef *storage_ExtFlashObj = NULL;
    storage_module_enable.val = 0;

    /* internal storage function still in developping */
    storage_module_enable.bit.internal = false;
    storage_module_enable.bit.external = FLASH_CHIP_STATE;

    if (storage_module_enable.bit.external)
    {
        storage_ExtFlashObj = SrvOsCommon.malloc(sizeof(Storage_ExtFLashDevObj_TypeDef));

        if (storage_ExtFlashObj)
        {
            storage_ExtFlashObj->bus_type = ExtFlash_Bus_Type;
            storage_ExtFlashObj->chip_type = ExtFlash_Chip_Type;
            storage_ExtFlashObj->dev_api = ExtFlash_Dev_Api;
            storage_ExtFlashObj->dev_obj = NULL;
        }
        else
        {
            SrvOsCommon.free(storage_ExtFlashObj);
            storage_module_enable.bit.external = false;
        }
    }

    while(1)
    {
        if (!init)
        {
            Storage.init(storage_module_enable, storage_ExtFlashObj);
            SrvComProto.init(SrvComProto_Type_MAV, NULL);
            
            TaskFrameCTL_Init(TaskFrameCTL_Period_Def);
            TaskBootCtl_Init(TaskBootCtl_Period_Def);

            osThreadDef(BootCTLTask, TaskBootCtl_Core, osPriorityNormal, 0, 1024);
            TaskBootCtl_Handle = osThreadCreate(osThread(BootCTLTask), NULL);

            osThreadDef(FrameCTLTask, TaskFrameCTL_Core, osPriorityAboveNormal, 0, 2048);
            TaskFrameCTL_Handle = osThreadCreate(osThread(FrameCTLTask), NULL);

            init = true;
        }

        /* run system statistic in this task */
        osDelay(10);
    }
}
