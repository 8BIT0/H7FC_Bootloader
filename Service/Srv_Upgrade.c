#include "Srv_Upgrade.h"
#include "util.h"
#include "../System/storage/Storage.h"
#include "Bsp_Flash.h"

#define DEFAULT_WINDOW_SIZE 100 /* unit: ms */
#define BootVer {0, 0, 0}
#define BootBref "First Version of H7FC Bootloader"
#define BootCompileData __DATA__
#define FIRMWARE_MAX_READ_SIZE (4 Kb)

/* get virable from .ld file defined */
extern uint32_t __rom_s;
extern uint32_t __rom_e;
extern uint32_t __boot_s;
extern uint32_t __boot_e;

typedef void (*Application_Func)(void);

typedef enum
{
    Stage_Get_FirmwareInfo = 0,
    Stage_Wait_DataInput,
    Stage_JumpAddr_Error,
} SrvUpgrade_State_List;

typedef enum
{
    UpgradeParam_None = 0,
    UpgradeParam_InValid,
    UpgradeParam_Valid,
} SrvUpgrade_ParamValid_List;

typedef struct
{
    SrvUpgrade_CodeStage_List CodeStage;
    SrvUpgrade_ParamValid_List ParamStatus;
    UpgradeInfo_TypeDef Info;
    
    SrvUpgrade_Send_Callback send;

    uint32_t firmware_addr_s;   /* Application or Module firmware storaged address start pos */
    uint32_t firmware_addr_e;   /* Application or Module firmware storaged address end pos */
    uint32_t firmware_size;     /* total firmware size */
    uint32_t firmware_rw_size;  /* current read or write size */

    SrvUpgrade_State_List UpgradeStage;
    uint32_t jump_time;
    uint32_t JumpAddr;

    uint8_t buf[FIRMWARE_MAX_READ_SIZE];
    uint16_t buf_size;

    uint8_t LogOut_Info[1024];
    uint16_t LogOut_Info_size;
} SrvUpgradeMonitor_TypeDef;

/* internal virable */
static SrvUpgradeMonitor_TypeDef Monitor;

/* internal function */

/* external function */
static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size);
static void SrvUpgrade_Set_Send(SrvUpgrade_Send_Callback callback);

/* external function */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .set_send_callback = SrvUpgrade_Set_Send,
};

static void SrvUpgrade_Set_Send(SrvUpgrade_Send_Callback callback)
{
    Monitor.send = callback;
}

static bool SrvUpgrade_Init(SrvUpgrade_CodeStage_List stage, uint32_t window_size)
{
    Storage_ItemSearchOut_TypeDef search_out;

    /* get data from storage */
    memset(&Monitor.Info, 0, sizeof(UpgradeInfo_TypeDef));
    search_out = Storage.search(External_Flash, Para_Boot, "Boot Info");
    if ((search_out.item_addr != 0) && \
        (Storage.get(External_Flash, Para_Boot, search_out.item, &Monitor.Info, sizeof(UpgradeInfo_TypeDef)) == Storage_Error_None))
    {
        Monitor.ParamStatus = UpgradeParam_None;
        switch ((uint8_t)stage)
        {
            case On_App:
                if (Monitor.Info.reg.bit.Boot || Monitor.Info.reg.bit.Module)
                    Monitor.ParamStatus = UpgradeParam_Valid;
                break;

            case On_Boot:
                if (Monitor.Info.reg.bit.App || Monitor.Info.reg.bit.Module)
                    Monitor.ParamStatus = UpgradeParam_Valid;
                break;

            default:
                Monitor.ParamStatus = UpgradeParam_InValid;
                return false;
        }
        
        Monitor.CodeStage = stage;
        return true;
    }
    
    Monitor.jump_time = SrvOsCommon.get_os_ms();
    if (window_size >= DEFAULT_WINDOW_SIZE)
    {
        Monitor.jump_time += window_size;
    }
    else
        Monitor.jump_time += DEFAULT_WINDOW_SIZE;
    Monitor.ParamStatus = UpgradeParam_InValid;
    return false;
}

static void SrvUpgrade_StatePolling(void)
{
    Storage_ItemSearchOut_TypeDef search_out;
    FrimwareInfo_TypeDef FrimInfo;

    memset(&search_out, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&FrimInfo, 0, sizeof(FrimwareInfo_TypeDef));

    if (Monitor.ParamStatus == UpgradeParam_Valid)
    {
        /* invalidation window time */
        if (Monitor.Info.reg.bit.App)
        {

        }
        else if (Monitor.Info.reg.bit.Module)
        {

        }
    }
    else
    {

    }
}

static void SrvUpgrade_Parse(uint8_t *p_buf, uint16_t len)
{
    if (p_buf && len)
    {

    }
}

static bool SrvUpgrade_CheckAppAddr(uint32_t addr, uint32_t size)
{
    /* check app base address */
    if ((addr & 0xFFFF0000) != ((uint32_t)&__rom_s))
        /* error address */
        return false;

    /* app start address check */
    if (addr < ((uint32_t)&__boot_e))
        /* app start address is lower then end of the boot section */
        return false;

    /* app size range check */
    if ((addr + size) > ((uint32_t)&__rom_e))
        /* end of app address is overrange */
        return false;

    return true;
}

static void SrvUpgrade_JumpTo(uint32_t addr, uint32_t app_size)
{
    uint32_t addr_tmp = addr;

    if (addr_tmp && app_size && SrvUpgrade_CheckAppAddr(addr_tmp, app_size))
    {
        /* log out jump addr and app size */

        /* disable all interrupt before jump to app */
        SrvOsCommon.disable_all_irq();

        Monitor.JumpAddr = *(volatile uint32_t *)(addr_tmp + 4);
        __set_MSP(*(volatile uint32_t *)addr_tmp);
        ((Application_Func)Monitor.JumpAddr)();
    }
    else
        Monitor.UpgradeStage = Stage_JumpAddr_Error;
}

