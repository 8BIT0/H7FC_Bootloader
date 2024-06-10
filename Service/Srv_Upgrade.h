#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define Max_App_Num 32

typedef enum
{
    Stage_Init = 0,
    Stage_Process_PortData,
    Stage_Upgrade_Error,
    Stage_ReadyToJump,
    Stage_Upgrade_Finish,
    Stage_Unknow,
} SrvUpgrade_Stage_List;

typedef enum
{
    PortProc_None = 0,
    PortProc_Deal_Pack,
    /* receive firmware pack stage */
    PortProc_Deal_Error,
    ProtProc_Finish,
    PortProc_Deal_TimeOut,
    PortProc_InValid_Data,
    PortProc_Unknown,
} SrvUpgrade_PortDataProc_List;

typedef struct
{
    SrvUpgrade_Stage_List stage;
    bool All_Port_Disabled;
} SrvUpgrade_State_TypeDef;

#pragma pack(1)
typedef union
{
    uint8_t val;

    struct
    {
        uint8_t Boot   : 1;
        uint8_t App    : 1;
        uint8_t res    : 6;
    } bit;
} SrvUpgrade_CTLReg_TypeDef;

typedef struct
{
    uint8_t File_Type;
    uint8_t Adapter_Type;
    uint8_t SW_Ver[3];
    uint8_t HW_Ver[3];
    uint32_t File_Size;
} FileInfo_TypeDef;

typedef struct
{
    SrvUpgrade_CTLReg_TypeDef CTLReg;
    FileInfo_TypeDef BF_Info;   /* boot firmware info */
    FileInfo_TypeDef AF_Info;   /* app firmware info */
} SrvUpgradeInfo_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(uint32_t window_size);
    SrvUpgrade_Stage_List (*polling)(uint32_t sys_time);
    void (*jump)(void);
    uint16_t (*get_log)(uint8_t *p_info, uint16_t len);
    void (*clear_log)(void);
} SrvUpgrade_TypeDef;

extern const uint8_t HWVer[3];
extern const uint8_t AppVer[3];
extern SrvUpgrade_TypeDef SrvUpgrade;

#endif
