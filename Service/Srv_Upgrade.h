#ifndef __SRV_UPGRADE_H
#define __SRV_UPGRADE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define MAX_BOOTLOADER_FRIMWARE_SIZE (64 * 1024)
#define Max_App_Num 32

typedef enum
{
    On_Boot = 0,
    On_App,
} SrvUpgrade_CodeStage_List;

typedef union
{
    uint8_t val;

    struct
    {
        uint8_t Boot    : 1;
        uint8_t App     : 1;
        uint8_t Module  : 1;
    } bit;
} UpgradeReg_TypeDef;

typedef enum
{
    FileType_None = 0,
    FileType_APP,
    FileType_Boot,
    FileType_Telemtry,
} Frimware_FileType_List;

typedef struct
{
    uint8_t  type;
    uint32_t tar_addr;
    uint32_t size;
    uint8_t ver[3];
} FrimwareInfo_TypeDef;

typedef enum
{
    Stage_Init = 0,
    Stage_Wait_PortData,
    Stage_Checking_App_Firmware,
    Stage_Checking_Module_Firmware,
    Stage_FirmwareData_Error,
    Stage_Processing_PortData,
    Stage_PortData_Error,
    Stage_Proto_TimeOut,
    Stage_App_Upgrading,
    Stage_Module_Upgrading,
    Stage_ReadyToJump,
    Stage_JumpToTarget,
    Stage_JumpError,
    Stage_Unknow,
} SrvUpgrade_Stage_List;

#pragma pack(1)
typedef struct
{
    UpgradeReg_TypeDef reg;
    uint32_t app_num;
    uint32_t app_addr_list[Max_App_Num];
    uint32_t jump_addr;
} UpgradeInfo_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
    SrvUpgrade_Stage_List (*polling)(void);
    void (*jump)(void);
    uint16_t (*get_log)(uint8_t *p_info, uint16_t len);
    void (*clear_log)(void);
} SrvUpgrade_TypeDef;

extern SrvUpgrade_TypeDef SrvUpgrade;

#endif
