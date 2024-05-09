#include "Task_Protocol.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "Srv_ComProto.h"
#include "Srv_FileAdapter.h"
#include "Srv_Upgrade.h"
#include "../FCHW_Config.h"
#include "DataPipe.h"
#if defined AT32F435RGT7
#include "../HW_Lib/AT32F435/HW_Def.h"
#elif defined STM32H743xx
#include "../HW_Lib/STM32H7/HW_Def.h"
#endif

DataPipe_CreateDataObj(SrvUpgrade_State_TypeDef, t_PortState);
static SrvUpgrade_State_TypeDef BootState = {
    .stage = Stage_Init,
};

#define PROTO_STREAM_BUF_SIZE 1024
#define VCP_CONNECT_TIMEOUT 50      /* unit: ms */

#if (RADIO_UART_NUM > 0)
static uint8_t RadioRxBuff[RADIO_UART_NUM][RADIO_BUFF_SIZE];

static BspUARTObj_TypeDef Radio_Port1_UartObj = {
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
    .rx_buf = RadioRxBuff[0],
    .rx_size = RADIO_BUFF_SIZE,
};

static FrameCTL_UartPortMonitor_TypeDef Radio_UartPort_List[RADIO_UART_NUM] = {
    [0] = {.init_state = false,
           .Obj = &Radio_Port1_UartObj},
};
#endif

/* internal variable */
static bool cli_state = false;
static bool Port_PipeUpdate = false;

/* MAVLink message List */


static SrvComProto_MsgObj_TypeDef *InUsePort_MavMsgInput_Obj;
static SrvComProto_MsgObj_TypeDef DefaultPort_MavMsgInput_Obj;
static SrvComProto_MsgObj_TypeDef RadioPort_MavMsgInput_Obj;

static FrameCTL_PortMonitor_TypeDef PortMonitor = {.init = false};
static uint32_t FrameCTL_Period = 0;
static __attribute__((section(".Perph_Section"))) uint8_t MavShareBuf[1024];
static __attribute__((section(".Perph_Section"))) uint8_t CLIRxBuf[CLI_FUNC_BUF_SIZE];
static uint8_t CLIProcBuf[CLI_FUNC_BUF_SIZE];
static uint8_t Uart_RxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint8_t Uart_TxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint8_t USB_RxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint32_t Radio_Addr = 0;
static uint32_t USB_VCP_Addr = 0;

static SrvComProto_Stream_TypeDef UartRx_Stream = {
    .p_buf = Uart_RxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(Uart_RxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef Log_Stream = {
    .p_buf = Uart_TxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(Uart_TxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef USBRx_Stream = {
    .p_buf = USB_RxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(USB_RxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef MavStream = {
    .p_buf = MavShareBuf,
    .size = 0,
    .max_size = sizeof(MavShareBuf),
};

static SrvComProto_Stream_TypeDef CLI_RX_Stream = {
    .p_buf = CLIRxBuf,
    .size = 0,
    .max_size = sizeof(CLIRxBuf),
};

static SrvComProto_Stream_TypeDef CLI_Proc_Stream = {
    .p_buf = CLIProcBuf,
    .size = 0,
    .max_size = sizeof(CLIProcBuf),
};

static FrameCTL_CLIMonitor_TypeDef CLI_Monitor = {
    .port_addr = 0,
    .p_rx_stream = &CLI_RX_Stream,
    .p_proc_stream = &CLI_Proc_Stream,
};

/* frame section */
static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size);

/* mavlinke frame decode callback */

/* pipe callback */
static void TaskPortCTL_PipeSendCallback(void *pipe_obj);

/* default vcp port section */
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint32_t *size);
static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp);
static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index);
static bool TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size);
static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_CLI_Proc(void);
static int TaskFrameCTL_CLI_Trans(const uint8_t *p_data, uint16_t size);
static bool TaskFrameCTL_Upgrade_Send(uint8_t *p_buf, uint16_t size);
static void TaskFrameCTL_ConfigureStateCheck(void);

/* upgrade proto section */
static void TaskFrameCTL_UpgradeInfo_Out(void);
static void TaskFrameCTL_Upgrade_StateCheck(void);

void TaskFrameCTL_Init(uint32_t period)
{
    FrameCTL_Period = FrameCTL_MAX_Period;

    /* USB VCP as defaut port to tune parameter and frame porotcol */
    memset(&PortMonitor, 0, sizeof(PortMonitor));
    memset(&DefaultPort_MavMsgInput_Obj, 0, sizeof(SrvComProto_MsgObj_TypeDef));
    memset(&RadioPort_MavMsgInput_Obj, 0, sizeof(SrvComProto_MsgObj_TypeDef));

    TaskFrameCTL_DefaultPort_Init(&PortMonitor);
    TaskFrameCTL_RadioPort_Init(&PortMonitor);
    Radio_Addr = TaskFrameCTL_Set_RadioPort(Port_Uart, 0);

    PortMonitor.init = true;
    
    if(period && (period <= FrameCTL_MAX_Period))
        FrameCTL_Period = period;

    JumpState_PortPipe.data_addr = DataPipe_DataObjAddr(t_PortState);
    JumpState_PortPipe.data_addr = DataPipe_DataSize(t_PortState);
    JumpState_PortPipe.trans_finish_cb = TaskPortCTL_PipeSendCallback;
    DataPipe_Enable(&JumpState_PortPipe);

    /* Shell Init */
    Shell_Init(TaskFrameCTL_CLI_Trans, CLI_Monitor.p_proc_stream->p_buf, CLI_Monitor.p_proc_stream->max_size);
}

void TaskFrameCTL_Core(void *arg)
{
    uint32_t per_time = SrvOsCommon.get_os_ms();
    uint8_t t_buf[] = "test\r\n";

    while(1)
    {
        /* check vcp connect state */
        TaskFrameCTL_ConfigureStateCheck();
        
        /* command line process */
        TaskFrameCTL_CLI_Proc();

        /* Log Out upgrade info */
        TaskFrameCTL_UpgradeInfo_Out();

        /* Port Process Pre Jump */
        TaskFrameCTL_Upgrade_StateCheck();

        SrvOsCommon.precise_delay(&per_time, FrameCTL_Period);
    }
}

/*************************** ByPass Mode is still in developping *********************************/
/************************************** radio section ********************************************/
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
        if(BspUSB_VCP.init((uint32_t)&(monitor->VCP_Port.RecObj)) != BspUSB_Error_None)
        {
            /* init default port VCP first */
            monitor->VCP_Port.init_state = false;
            return;
        }
        else
            monitor->VCP_Port.init_state = true;

        /* create USB VCP Tx semaphore */
        osSemaphoreDef(DefaultPort_Tx);
        monitor->VCP_Port.p_tx_semphr = osSemaphoreCreate(osSemaphore(DefaultPort_Tx), 1);

        if(monitor->VCP_Port.p_tx_semphr == NULL)
        {
            monitor->VCP_Port.init_state = false;
            return;
        }

        BspUSB_VCP.set_tx_cpl_callback(TaskFrameCTL_Port_TxCplt_Callback);
        BspUSB_VCP.set_rx_callback(TaskFrameCTL_Port_Rx_Callback);
        BspUSB_VCP.set_connect_callback(TaskFrameCTL_USB_VCP_Connect_Callback);

        monitor->VCP_Port.RecObj.PortObj_addr = (uint32_t)&(monitor->VCP_Port);

        USB_VCP_Addr = (uint32_t)&(monitor->VCP_Port);
    }
}

static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size)
{
    bool state = false;

    /* when attach to host device then send data */
    if (PortMonitor.VCP_Port.init_state && \
        PortMonitor.vcp_connect_state && \
        PortMonitor.VCP_Port.p_tx_semphr && \
        p_data && size)
    {
        state = true;
        osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, 0);
        if ((BspUSB_VCP.send(p_data, size) != BspUSB_Error_None) || \
            (osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, FrameCTL_Port_Tx_TimeOut) < 0))
            state = false;
    }

    return state;
}

/************************************** radio port section *************************/
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
#if (RADIO_UART_NUM > 0)
        monitor->uart_port_num = RADIO_UART_NUM;
        monitor->Uart_Port = Radio_UartPort_List;
        
        for(uint8_t i = 0; i < monitor->uart_port_num; i++)
        {
#if defined STM32H743xx
            /* create port obj element */
            monitor->Uart_Port[i].Obj->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
            if(monitor->Uart_Port[i].Obj->hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }

            monitor->Uart_Port[i].Obj->rx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->rx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }

            monitor->Uart_Port[i].Obj->tx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->tx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->tx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return;
            }
#endif

            if(BspUart.init(monitor->Uart_Port[i].Obj))
            {
                monitor->Uart_Port[i].init_state = true;
                memset(&monitor->Uart_Port[i].RecObj, 0, sizeof(FrameCTL_PortProtoObj_TypeDef));
                memset(&monitor->Uart_Port[i].ByPass_Mode, 0, sizeof(Port_Bypass_TypeDef));
                
                monitor->Uart_Port[i].RecObj.type = Port_Uart;
                monitor->Uart_Port[i].RecObj.port_index = i;
                
                monitor->Uart_Port[i].Obj->cust_data_addr = (uint32_t)&(monitor->Uart_Port[i].RecObj);
            
                /* create semaphore for send */
                osSemaphoreDef(Uart_Port_Tmp);
                monitor->Uart_Port[i].p_tx_semphr = osSemaphoreCreate(osSemaphore(Uart_Port_Tmp), 1);

                if(monitor->Uart_Port[i].p_tx_semphr)
                {
                    /* set callback */
                    BspUart.set_rx_callback(monitor->Uart_Port[i].Obj, TaskFrameCTL_Port_Rx_Callback);
                    BspUart.set_tx_callback(monitor->Uart_Port[i].Obj, TaskFrameCTL_Port_TxCplt_Callback);

                    monitor->Uart_Port[i].RecObj.PortObj_addr = (uint32_t)&(monitor->Uart_Port[i]);
                    monitor->Uart_Port[i].init_state = true;
                }
                else
                {
                    monitor->Uart_Port[i].init_state = false;
                    monitor->uart_port_num --;
                }
            }
            else
            {
                monitor->Uart_Port[i].init_state = false;
                monitor->uart_port_num --;
            }
        }
#else
        monitor->uart_port_num = 0;
#endif
    }
}

static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index)
{
    uint32_t port_hdl = 0;

    switch((uint8_t) port_type)
    {
        case Port_Uart:
            if((index < PortMonitor.uart_port_num) && PortMonitor.Uart_Port[index].init_state)
            {
                port_hdl = (uint32_t)&(PortMonitor.Uart_Port[index]);
            }
            break;

        default:
            return port_hdl;
    }

    return port_hdl;
}

/************************************** receive process callback section *************************/
static bool TaskFrameCTL_Port_DeInit(void)
{
    bool state = false;
    static bool vcp_state = false;
    static uint8_t port_index = 0;

    /* disable usb vcp */
    if (!vcp_state || (BspUSB_VCP.de_init() == BspUSB_Error_None))
        vcp_state = true;

    /* disable radio port */
    for (; port_index < RADIO_UART_NUM; port_index ++)
    {
        // BspUart.de_init();
    }

    if (vcp_state && (port_index == RADIO_UART_NUM))
        return true;

    return false;
}

static bool TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size)
{
    bool state = false;    
    FrameCTL_UartPortMonitor_TypeDef *p_UartPort = NULL;

    if(obj_addr && p_data && size)
    {
        state = true;
        p_UartPort = (FrameCTL_UartPortMonitor_TypeDef *)obj_addr;

        if(p_UartPort->init_state && p_UartPort->Obj && p_UartPort->p_tx_semphr)
        {
            osSemaphoreWait(p_UartPort->p_tx_semphr, 0);
            if ((!BspUart.send(p_UartPort->Obj, p_data, size)) || \
                (osSemaphoreWait(p_UartPort->p_tx_semphr, FrameCTL_Port_Tx_TimeOut) != osOK))
                state = false;
        }
        else
            state = false;
    }

    return state;
}

static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    SrvComProto_Stream_TypeDef *p_stream = NULL;
    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;
    InUsePort_MavMsgInput_Obj = NULL;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
        p_RecObj = (FrameCTL_PortProtoObj_TypeDef *)RecObj_addr;
        p_RecObj->time_stamp = SrvOsCommon.get_os_ms();

        switch((uint8_t) p_RecObj->type)
        {
            case Port_USB:
                p_stream = &USBRx_Stream;
                InUsePort_MavMsgInput_Obj = &DefaultPort_MavMsgInput_Obj;

                if (cli_state)
                    TaskFrameCTL_DefaultPort_Trans(p_data, size);
                break;

            case Port_Uart:
                p_stream = &UartRx_Stream;
                InUsePort_MavMsgInput_Obj = &RadioPort_MavMsgInput_Obj;
                
                if (cli_state)
                    TaskFrameCTL_Port_Tx(p_RecObj->PortObj_addr, p_data, size);
                break;

            default:
                return;
        }

        if ((p_stream->size + size) <= p_stream->max_size)
        {
            memcpy(p_stream->p_buf + p_stream->size, p_data, size);
            p_stream->size += size;
        }
        else
        {
            memset(p_stream->p_buf, 0, p_stream->size);
            p_stream->size = 0;
        }

        stream_in = SrvComProto.msg_decode(InUsePort_MavMsgInput_Obj, p_stream->p_buf, p_stream->size);
    
        /* noticed when drone is under disarmed state we can`t tune or send cli to drone for safety */
        if(stream_in.valid)
        {
            /* tag on recive time stamp */
            /* first come first serve */
            /* in case two different port tuning the same function or same parameter at the same time */
            /* if attach to configrator or in tunning then lock moto */
            if(!cli_state)
            {
                if (stream_in.pac_type == ComFrame_MavMsg)
                {
                    /* check mavlink message frame type */
                    /* only process mavlink message when cli is disabled */

                    /* after mavlink message processed */
                    /* deal with stream buffer */
                    if (p_stream->size > stream_in.size)
                    {
                        memmove(p_stream->p_buf, (stream_in.p_buf + stream_in.size), (p_stream->size - stream_in.size));
                    }
                    else
                        memset(p_stream->p_buf, 0, p_stream->size);

                    p_stream->size -= stream_in.size;
                }
            }
            
            if(stream_in.pac_type == ComFrame_CLI)
            {
                /* set current mode as cli mode */
                /* all command line end up with "\r\n" */
                /* push string into cli shared stream */
                if(((CLI_Monitor.port_addr == 0) || (CLI_Monitor.port_addr == p_RecObj->PortObj_addr)) && \
                   (CLI_Monitor.p_rx_stream->size + p_stream->size) <= CLI_Monitor.p_rx_stream->max_size)
                {
                    cli_state = true;
                    CLI_Monitor.type = p_RecObj->type;
                    CLI_Monitor.port_addr = p_RecObj->PortObj_addr;
                    memcpy(CLI_Monitor.p_rx_stream->p_buf + CLI_Monitor.p_rx_stream->size, stream_in.p_buf, stream_in.size);
                    CLI_Monitor.p_rx_stream->size += p_stream->size;
                }
            }
        
            memset(p_stream->p_buf, 0, p_stream->max_size);
        }
    }
}

static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t Obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;
    FrameCTL_UartPortMonitor_TypeDef *p_UartPortObj = NULL;
    FrameCTL_VCPPortMonitor_TypeDef *p_USBPortObj = NULL;
    osSemaphoreId semID = NULL;
    uint32_t *p_rls_err_cnt = NULL;

    if(Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if(p_Obj->PortObj_addr)
        {
            switch((uint8_t) p_Obj->type)
            {
                case Port_USB:
                    p_USBPortObj = (FrameCTL_VCPPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_USBPortObj->init_state && p_USBPortObj->p_tx_semphr)
                    {
                        semID = p_USBPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_USBPortObj->tx_semphr_rls_err;
                    }
                    break;

                case Port_Uart:
                    p_UartPortObj = (FrameCTL_UartPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_UartPortObj->init_state && p_UartPortObj->p_tx_semphr)
                    {
                        semID = p_UartPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_UartPortObj->tx_semphr_rls_err;
                    }
                    break;

                default:
                    return;
            }
            
            if(semID && p_rls_err_cnt && (osSemaphoreRelease(semID) != osOK))
                (*p_rls_err_cnt) ++;
        }
    }
}

/************************************** USB Only Callback section ********************************************/
/*
 * use usb sof interrupt
 * as long as usb port attach to computer or other host device
 * flight controller will receive sof interrupt
 * if flight controller continue to receive this interrupt it must be attach to computer
 */
static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp)
{
    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;

    if (Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if (p_Obj->PortObj_addr && (p_Obj->type == Port_USB))
            *time_stamp = SrvOsCommon.get_os_ms();
    }
}
/************************************** USB Only Callback section ********************************************/

static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size)
{
    if(Obj && (Obj->frame_type == ComFrame_MavMsg) && Obj->port_addr && p_data && size)
    {
        switch((uint8_t)(Obj->port_type))
        {
            case Port_Uart:
                TaskFrameCTL_Port_Tx(Obj->port_addr, p_data, size);
                break;

            case Port_USB:
                TaskFrameCTL_DefaultPort_Trans(p_data, size);
                break;

            default:
                return;
        }
    }
}
/***************************************** Frame mavlink Receive Callback ************************************/
static void TaskFrameCTL_ConfigureStateCheck(void)
{
    uint32_t tunning_time_stamp = 0;
    uint32_t tunning_port = 0;
    bool tunning_state = false;
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    bool lst_vcp_state = false;

    PortMonitor.vcp_connect_state = false;

    /* check usb vcp attach state */
    if (BspUSB_VCP.check_connect)
        PortMonitor.vcp_connect_state = BspUSB_VCP.check_connect(cur_time, FrameCTL_Period);
}

/***************************************** Pipe Callback ************************************************/
static void TaskPortCTL_PipeSendCallback(void *pipe_obj)
{
    if (pipe_obj == &JumpState_PortPipe)
    {
        Port_PipeUpdate = true;
    }
}

/***************************************** Upgrade Section ***********************************************/
static bool TaskFrameCTL_Upgrade_Send(uint8_t *p_buf, uint16_t size)
{
    bool state = false;

    /* transmit via usb vcp port */
    // state = TaskFrameCTL_DefaultPort_Trans(p_buf, size);

    /* transmit via uart port */
    for (uint8_t i = 0; i < PortMonitor.uart_port_num; i++)
    {
        state |= TaskFrameCTL_Port_Tx(PortMonitor.Uart_Port[i].RecObj.PortObj_addr, p_buf, size);
    }

    return state;
}

static void TaskFrameCTL_UpgradeInfo_Out(void)
{
    uint16_t log_size = 0;
    
    /* check enbal port type */

    log_size = SrvUpgrade.get_log((Log_Stream.p_buf + Log_Stream.size), Log_Stream.max_size);
    if (log_size || Log_Stream.size)
    {
        Log_Stream.size += log_size;
        if (TaskFrameCTL_Upgrade_Send(Log_Stream.p_buf, Log_Stream.size))
        {
            memset(Log_Stream.p_buf, 0, Log_Stream.size);
            Log_Stream.size = 0;
            SrvUpgrade.clear_log();
        }
    }
}

static void TaskFrameCTL_Upgrade_StateCheck(void)
{
    if (Port_PipeUpdate)
    {
        switch ((uint8_t)DataPipe_DataObj(t_PortState).stage)
        {
            case Stage_ReadyToJump:
                /* disable all port */
                if (TaskFrameCTL_Port_DeInit())
                {
                    DataPipe_DataObj(t_PortState).All_Port_Disabled = true;
                    DataPipe_SendTo(&JumpState_PortPipe, &JumpState_BootPipe);
                }
                break;
            
            default: break;
        }

        Port_PipeUpdate = false;
    }
}

/***************************************** CLI Section ***********************************************/
static void TaskFrameCTL_CLI_Proc(void)
{
    uint16_t rx_stream_size = 0;
    Shell *shell_obj = Shell_GetInstence();

    /* check CLI stream */
    if(shell_obj && CLI_Monitor.p_rx_stream->p_buf && CLI_Monitor.p_rx_stream->size)
    {
        rx_stream_size = CLI_Monitor.p_rx_stream->size;

        for(uint16_t i = 0; i < rx_stream_size; i++)
        {
            shellHandler(shell_obj, CLI_Monitor.p_rx_stream->p_buf[i]);
            CLI_Monitor.p_rx_stream->p_buf[i] = 0;
            CLI_Monitor.p_rx_stream->size --;
        }
    }
}

static int TaskFrameCTL_CLI_Trans(const uint8_t *p_data, uint16_t size)
{
    if(p_data && size)
    {
        switch ((uint8_t) CLI_Monitor.type)
        {
            case Port_Uart:
                TaskFrameCTL_Port_Tx(CLI_Monitor.port_addr, (uint8_t *)p_data, size);
                break;
            
            case Port_USB:
                TaskFrameCTL_DefaultPort_Trans((uint8_t *)p_data, size);
                break; 

            default:
                break;
        }
    }

    return 0;
}

static void TaskFermeCTL_CLI_DisableControl(void)
{
    Shell *shell_obj = Shell_GetInstence();
    
    if (shell_obj)
    {
        shellPrint(shell_obj, "\r\n\r\n");
        shellPrint(shell_obj, "CLI Disabled\r\n");
        cli_state = false;
        CLI_Monitor.port_addr = 0;
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, CLI_Disable,  TaskFermeCTL_CLI_DisableControl, CLI Enable Control);

static void TaskFrameCTL_FileAccept()
{
    Shell *shell_obj = Shell_GetInstence();

    if (shell_obj)
    {

    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Wait_File, TaskFrameCTL_FileAccept, In File Receive Mode);

