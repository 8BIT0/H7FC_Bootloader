#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "Bsp_Uart.h"

#define To_DataPack_Callback(x) (DataPack_Callback)x

/* only can use one hardware port at one time */
/* still can be optmize / use multi port proto mavlink frame */

SrvComProto_Monitor_TypeDef SrvComProto_monitor = {
    .Proto_Type = SrvComProto_Type_None,
    .init_state = false,
};

/* internal function */

/* just fot temporary will create custom message in the next */
static uint16_t SrvComProto_MavMsg_Exp_Attitude(SrvComProto_MsgInfo_TypeDef *pck);

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb);
static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state);
static SrvComProto_Type_List Srv_ComProto_GetType(void);
static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_DecodeAll(SrvComProto_MsgObj_TypeDef *obj, uint8_t *p_data, uint16_t size);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .mav_msg_obj_init = Srv_ComProto_MsgObj_Init,
    .get_msg_type = Srv_ComProto_GetType,
    .mav_msg_stream = SrvComProto_MsgToStream,
    .mav_msg_enable_ctl = SrvComProto_MsgEnable_Control,
    .msg_decode = SrvComProto_MavMsg_Input_DecodeAll,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    // UNUSED(arg);

    /* only init one time */
    if (SrvComProto_monitor.init_state)
        return true;

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;
    SrvComProto_monitor.init_state = true;

    return true;
}

static SrvComProto_Type_List Srv_ComProto_GetType(void)
{
    return SrvComProto_monitor.Proto_Type;
}

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period)
{
    if ((msg == NULL) || \
        (period == 0))
        return false;

    msg->in_proto = false;
    msg->lock_proto = true;

    msg->pck_info = pck_info;
    msg->period = period;
    msg->proto_time = 0;

    /* create mavlink message object */
    msg->msg_obj = (mavlink_message_t *)SrvOsCommon.malloc(sizeof(mavlink_message_t));

    if (msg->msg_obj == NULL)
    {
        SrvOsCommon.free(msg->msg_obj);
        return false;
    }

    memset(msg->msg_obj, 0, sizeof(mavlink_message_t));

    /* set mavlink data structure value set function */
    switch ((uint8_t)pck_info.component_id)
    {
        default:
            SrvOsCommon.free(msg->msg_obj);
            msg->lock_proto = false;
            return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    if (msg->enable && com_stream && com_stream->p_buf && msg->pack_callback)
    {
        msg->in_proto = true;

        if ((msg->proto_time) && (sys_time - msg->proto_time < msg->period))
        {
            msg->in_proto = false;
            return;
        }

        com_stream->size = msg->pack_callback((uint8_t *)msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg->msg_obj);

            if (tx_cb)
            {
                tx_cb(arg, com_stream->p_buf, com_stream->size);
                msg->proto_cnt ++;
                memset(com_stream->p_buf, 0, com_stream->size);
            }

            msg->proto_time = sys_time;
        }

        msg->in_proto = false;
    }
}

static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state)
{
    if (msg == NULL)
        return false;

    msg->enable = state;
    return true;
}

/******************************************* Frame Out ****************************************/

static void SrvComProto_Set_MavMsgIn_Callback(SrvComProto_MsgObj_TypeDef *obj, SrvComProto_MavInMsgType_List type, SrvComProto_MavMsgIn_Callback callback, void *p_cus_data)
{
    if (obj && p_cus_data)
    {
        switch ((uint8_t)type)
        {
            default: break;
        }
    }
}

static bool SrvComProto_MavMsg_Decode_ParamOperation(SrvComProto_MsgObj_TypeDef *obj, const mavlink_message_t msg)
{
    mavlink_para_operation_t msg_para_operation;

    if (obj)
    {
        obj->Para_Operation_Cnt ++;

        memset(&msg_para_operation, 0, sizeof(mavlink_para_operation_t));
        mavlink_msg_para_operation_decode(&msg, &msg_para_operation);
    
        if (obj->MavMsg_ParamOperation_Callback)
            return obj->MavMsg_ParamOperation_Callback(&msg_para_operation, obj->cus_p_paraoperation);
    }

    return false;
}

/******************************************* Frame In  ****************************************/
static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_DecodeAll(SrvComProto_MsgObj_TypeDef *obj, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in; 
    uint8_t default_channel = 0;
    mavlink_message_t mav_msg;
    mavlink_status_t mav_sta;
    volatile uint8_t mav_decode = 0;
    // bool decode_state = false;
    
    memset(&stream_in, 0, sizeof(SrvComProto_Msg_StreamIn_TypeDef));

    /* match cli */
    if((p_data[size - 1] == '\n') && (p_data[size - 2] == '\r'))
    {
        stream_in.pac_type = ComFrame_CLI;
        stream_in.valid = true;
        stream_in.size = size;
        stream_in.p_buf = p_data;
    
        goto input_stream_valid;
    }
    
    memset(&mav_msg, 0, sizeof(mavlink_message_t));
    memset(&mav_sta, 0, sizeof(mavlink_status_t));
    
    /* match mavlink message */
    for(uint16_t i = 0; i < size; i++)
    {
        mav_decode = mavlink_frame_char(default_channel, p_data[i], &mav_msg, &mav_sta);

        if (mav_decode == MAVLINK_FRAMING_OK)
        {
            /* get multi protocol frame in one transmition and first protocol frame is mavlink message */
            /* only decode first pack */
            if (obj && (mav_msg.sysid == MAV_SysID_Radio))
            {
                switch ((uint8_t)mav_msg.compid)
                {
                    default: break;
                }
            }
            
            /* mavlink frame ack */
            // SrvComProto_MavMsg_Ack(obj, MAV_SysID_Drone, uint16_t compo_id, decode_state);

            memset(&mav_msg, 0, sizeof(mavlink_message_t));
            memset(&mav_sta, 0, sizeof(mavlink_status_t));
        }
        else if (mav_decode == MAVLINK_FRAMING_BAD_CRC)
        {
            memset(&mav_msg, 0, sizeof(mavlink_message_t));
            memset(&mav_sta, 0, sizeof(mavlink_status_t));
        }
    }

    /* custom frame input check */
    stream_in.pac_type = ComFrame_MavMsg;
    stream_in.valid = true;
    stream_in.size = size;
    stream_in.p_buf = p_data;

input_stream_valid:
    return stream_in;
}
