#include "Task_BootCtl.h"
#include "Srv_OsCommon.h"
#include "Srv_Upgrade.h"
#include "Storage.h"
#include "Bsp_Uart.h"
#include "Bsp_USB.h"
#include "Bsp_DMA.h"
#if defined AT32F435RGT7
#include "../HW_Lib/AT32F435/HW_Def.h"
#elif defined STM32H743xx
#include "../HW_Lib/STM32H7/HW_Def.h"
#else
#include "../HW_Lib/AT32F435/HW_Def.h"
#endif

#define RADIO_BUFF_SIZE 512
static uint8_t RadioRxBuff[RADIO_BUFF_SIZE];

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

typedef enum
{
    Port_Bypass_None = 0,
    Port_Bypass_RxOnly,
    Port_Bypass_TxOnly,
    Port_Bypass_BothDir,
} Bypass_TypeList;

typedef struct
{
    bool enable;
    Bypass_TypeList bypass_mode;
    uint8_t *bypass_src;
} Port_Bypass_TypeDef;

typedef struct
{
    BootPort_Type_List type;
    uint8_t port_index;
    uint32_t Obj_addr;
    uint32_t time_stamp;
} BootPortProtoObj_TypeDef;

typedef struct
{
    uint32_t period;
    uint32_t jump_time;
    Boot_UpdateType_List update_type;

    bool init_state;

    BootPortProtoObj_TypeDef port_obj;
    uint8_t avaliable_port_num;
} BootCtlMonitor_TypeDef;

typedef struct
{
    bool init_state;
    BootPortProtoObj_TypeDef RecObj;
    Port_Bypass_TypeDef ByPass_Mode;
    
    osSemaphoreId p_tx_semphr;
    uint32_t tx_semphr_rls_err;

    BspUARTObj_TypeDef *Obj;
} BootCtl_UartPortMonitor_TypeDef;

/* internal vriable */
static BootCtlMonitor_TypeDef BootMonitor = {
    .init_state = false,
    .avaliable_port_num = 0,
    .port_obj = {
        .Obj_addr = 0,
        .type = BootPort_None,
    },
};

BspUARTObj_TypeDef RadioPortObj = {
    .instance = RADIO_PORT,
    .baudrate = RADIO_PORT_BAUD,
    .tx_io = {
        .init_state = RADIO_TX_PIN_INIT_STATE,
        .pin = RADIO_TX_PIN,
        .port = RADIO_TX_PORT,
        .alternate = RADIO_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = RADIO_RX_PIN_INIT_STATE,
        .pin = RADIO_RX_PIN,
        .port = RADIO_RX_PORT,
        .alternate = RADIO_RX_PIN_ALT,
    }, 
    .pin_swap = false,
    .rx_dma = RADIO_RX_DMA,
    .rx_stream = RADIO_RX_DMA_STREAM,
    .tx_dma = RADIO_TX_DMA,
    .tx_stream = RADIO_TX_DMA_STREAM,
    .rx_buf = RadioRxBuff,
    .rx_size = RADIO_BUFF_SIZE,
};

/* internal function */
void TaskBootCtl_UartPort_Tx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);
void TaskBootCtl_UartPort_Rx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);

void TaskBootCtl_Init(uint32_t period)
{
    SrvUpgrade.init(RunningStage, 500);

    /* port init */
    if (BspUSB_VCP.init(&BootMonitor.port_obj))
        BootMonitor.avaliable_port_num ++;

    if (BspUart.init(&RadioPortObj))
    {
        BootMonitor.avaliable_port_num ++;
        BspUart.set_tx_callback(&RadioPortObj, TaskBootCtl_UartPort_Tx_Callback);
        BspUart.set_rx_callback(&RadioPortObj, TaskBootCtl_UartPort_Rx_Callback);
    }

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

        if (p_Obj->Obj_addr && (p_Obj->type == BootPort_USB))
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

/* Radio Port Callback */
void TaskBootCtl_UartPort_Tx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size)
{
    if (cust_data_addr && buff && size)
    {

    }
}

void TaskBootCtl_UartPort_Rx_Callback(uint32_t cust_data_addr, uint8_t *buff, uint16_t size)
{
    if (cust_data_addr && buff && size)
    {

    }
}

