/*
 *  Auther: 8_B!T0
 *  Noticed: this file use to upgrade App / Bootloader / Module Firmware 
 *  When at bootloader use this file can upgrade App and Module Firmware
 *  When at app use this file can upgrade Bootloader and Module Firmware
 *  
 *                      F ------- Y ------ I
 *                      still in developping
 */
#include "Srv_Upgrade.h"
#include "util.h"
#include "../System/storage/Storage.h"
#include "Bsp_Flash.h"
#include "../FCHW_Config.h"

#define FIRMWARE_WAITTING_TIMEOUT   60000   /* unit: ms */
#define FIRMWARE_COMMU_TIMEOUT      1000    /* unit: ms */
#define DEFAULT_WINDOW_SIZE         100     /* unit: ms */

const uint8_t BootVer[3] = {0, 0, 1};
#if defined MATEKH743_V1_5
const uint8_t HWVer[3] = {0, 0, 1};
#elif defined BATEAT32F435_AIO
const uint8_t HWVer[3] = {0, 0, 2};
#endif
#define AppBref "First Version of H7FC"
#define AppCompileData __DATA__
#define FIRMWARE_MAX_READ_SIZE (4 Kb)

#define UpgradeInfo_Sec  "Upgrade_Info"

typedef void (*Application_Func)(void);

static uint8_t upgrade_buf[FIRMWARE_MAX_READ_SIZE] = {0};

__attribute__((weak)) bool Write_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}
__attribute__((weak)) bool Read_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}
__attribute__((weak)) bool Erase_OnChipFlash(uint32_t addr, uint8_t *p_data, uint16_t size){return false;}

typedef enum
{
    UpgradeParam_None = 0,
    UpgradeParam_InValid,
    UpgradeParam_Valid,
} SrvUpgrade_ParamValid_List;

typedef struct
{
    bool init_state;

    SrvUpgrade_Stage_List PollingState;
    SrvUpgrade_PortDataProc_List PortDataState;
    
    /* useless in app */
    uint32_t jump_time;
    uint32_t JumpAddr;
    uint32_t AppSize;

    Storage_ItemSearchOut_TypeDef UpgradeInfo_SO;

    uint8_t LogOut_Info[1024];
    uint16_t LogOut_Info_size;
} SrvUpgradeMonitor_TypeDef;

typedef struct
{
    uint32_t read_addr;
    uint32_t read_size;

    uint32_t write_addr;
    uint32_t write_size;

    uint32_t remain_size;
} FlashMonitor_TypeDef;

/* internal virable */
static SrvUpgradeMonitor_TypeDef Monitor = {
    .init_state = false,
};

/* internal function */
static void SrvUpgrade_Collect_Info(const char *format, ...);
static void SrvUpgrade_CheckUpgrade_OnBootUp(void);

/* external function */
static bool SrvUpgrade_Init(uint32_t window_size);
static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time);
static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len);
static void SrvUpgrade_ClearLog(void);
static void SrvUpgrade_JumpTo(void);

/* external function */
SrvUpgrade_TypeDef SrvUpgrade = {
    .init = SrvUpgrade_Init,
    .polling = SrvUpgrade_StatePolling,
    .jump = SrvUpgrade_JumpTo,
    .get_log = SrvUpgrade_Get_Info,
    .clear_log = SrvUpgrade_ClearLog,
};

static bool SrvUpgrade_Init(uint32_t window_size)
{
    if (sizeof(upgrade_buf) % 2)
        return false;

    /* get data from storage */
    Monitor.LogOut_Info_size = 0;

    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;

    SrvUpgrade_Collect_Info("[SrvUpgrade Init]\r\n");
    SrvUpgrade_Collect_Info("\tOn Boot Stage\r\n");
    SrvUpgrade_Collect_Info("\tReading [Boot Info] from storage\r\n");
    SrvUpgrade_Collect_Info("\tBoot Version %d.%d.%d\r\n", BootVer[0], BootVer[1], BootVer[2]);
    
    Monitor.JumpAddr = App_Address_Base;
    Monitor.AppSize  = App_Section_Size;
    Monitor.jump_time = SrvOsCommon.get_os_ms();
    Monitor.jump_time += DEFAULT_WINDOW_SIZE;

    /* show jump time stamp */
    SrvUpgrade_Collect_Info("\tJump time: %d\r\n", Monitor.jump_time);
    SrvUpgrade_Collect_Info("\r\n");
    
    Monitor.init_state = true;
    Monitor.PollingState = Stage_Init;
    Monitor.PortDataState = PortProc_None;
    
    /* 
     * whatever on boot or app
     * check upgrade on boot up
     */
    SrvUpgrade_CheckUpgrade_OnBootUp();
    return true;
}

static void SrvUpgrade_CheckUpgrade_OnBootUp(void)
{
    SrvUpgradeInfo_TypeDef Info;
    uint32_t file_size = 0;
    uint16_t update_size = 0;
    uint32_t addr_offset = 0;

    memset(&Monitor.UpgradeInfo_SO, 0, sizeof(Monitor.UpgradeInfo_SO));
    memset(&Info, 0, sizeof(Info));

    /* check upgrade enable control first */
    Monitor.UpgradeInfo_SO = Storage.search(Para_Boot, UpgradeInfo_Sec);
    if (Monitor.UpgradeInfo_SO.item_addr)
    {
        Storage.get(Para_Boot, Monitor.UpgradeInfo_SO.item, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef));

        /* read parameter section */
        /* read boot firmware info */
        if (Info.CTLReg.bit.App)
        {
            /* check hardware version */
            if (memcmp(Info.AF_Info.HW_Ver, HWVer, sizeof(HWVer)) == 0)
            {
                SrvUpgrade_Collect_Info("[ Upgrading App ]\r\n");
                /* check app upgrade */
                file_size = Info.AF_Info.File_Size;
            }
        }
        
        while (file_size)
        {
            update_size = file_size;
            if (file_size > 1024)
                update_size = 1024;

            /* read firmware from storage */
            memset(upgrade_buf, 0, update_size);
            Storage.read_firmware(Firmware_App, addr_offset, upgrade_buf, update_size);

            SrvOsCommon.enter_critical();
            /* write firmware to boot flash */
            Storage.write_firmware(Internal_Flash, Firmware_App, addr_offset, upgrade_buf, update_size);
            SrvOsCommon.exit_critical();

            file_size -= update_size;
            addr_offset += update_size;
            
            if (file_size == 0)
            {
                /* clear upgrade flag */
                Info.CTLReg.bit.App = false;
                Storage.update(Para_Boot, Monitor.UpgradeInfo_SO.item.data_addr, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef));
                
                SrvUpgrade_Collect_Info("[ Upgrading Accomplished ]\r\n");
                break;
            }
        }
    }
    else
    {
        memcpy(Info.AF_Info.HW_Ver, HWVer, sizeof(Info.AF_Info.HW_Ver));
        memcpy(Info.BF_Info.HW_Ver, HWVer, sizeof(Info.AF_Info.HW_Ver));
        Storage.create(Para_Boot, UpgradeInfo_Sec, (uint8_t *)(&Info), sizeof(SrvUpgradeInfo_TypeDef));
        Monitor.UpgradeInfo_SO = Storage.search(Para_Boot, UpgradeInfo_Sec);
    }

    if (Monitor.UpgradeInfo_SO.item_addr == 0)
        Monitor.init_state = false;
}

static SrvUpgrade_Stage_List SrvUpgrade_StatePolling(uint32_t sys_time)
{
    switch ((uint8_t) Monitor.PollingState)
    {
        case Stage_Init:            
            Monitor.PollingState = Stage_Process_PortData;
            return Monitor.PollingState;

        case Stage_Process_PortData:
            if (sys_time >= Monitor.jump_time)
            {
                Monitor.PollingState = Stage_ReadyToJump;
                SrvUpgrade_Collect_Info("[Jump Preparetion]\r\n");
                SrvUpgrade_Collect_Info("\tDisabling Irq\r\n");

                return Stage_ReadyToJump;
            }
            return Stage_Process_PortData;

        /* when at bootloader */
        case Stage_ReadyToJump:
            return Stage_ReadyToJump;

        default: return Stage_Unknow;
    }
}

static bool SrvUpgrade_CheckAppAddr(uint32_t addr, uint32_t size)
{
    /* check app base address */
    if ((addr & 0xFF000000) != ((uint32_t)&__rom_s))
    {
        SrvUpgrade_Collect_Info("[ ------- Bad Jump Address 0x%08x -------]\r\n", addr);
        SrvUpgrade_Collect_Info("\r\n");
        /* error address */
        return false;
    }

    /* app start address check */
    if (addr < ((uint32_t)&__boot_e))
    {
        SrvUpgrade_Collect_Info("[ ------- Bad Jump Address 0x%08x ------- ]\r\n", addr);
        SrvUpgrade_Collect_Info("\r\n");
        /* app start address is lower then end of the boot section */
        return false;
    }

    /* app size range check */
    if ((addr + size) > ((uint32_t)&__rom_e))
    {
        SrvUpgrade_Collect_Info("[ ------- App Size Over Range ------- ]\r\n");
        SrvUpgrade_Collect_Info("\tApp Start Addr: 0x%08x\r\n", addr);
        SrvUpgrade_Collect_Info("\tApp End   Addr: 0x%08x\r\n", (addr + size));
        SrvUpgrade_Collect_Info("\tApp Size:       %d\r\n", size);
        SrvUpgrade_Collect_Info("\tFlash End Addr: 0x%08x\r\n", (uint32_t)&__rom_e);
        SrvUpgrade_Collect_Info("\r\n");
        /* end of app address is overrange */
        return false;
    }

    return true;
}

static void SrvUpgrade_JumpTo(void)
{
    uint32_t jump_addr = 0;

    if (Monitor.JumpAddr && Monitor.AppSize && SrvUpgrade_CheckAppAddr(Monitor.JumpAddr, Monitor.AppSize))
    {
        /* log out jump addr and app size */
        SrvUpgrade_Collect_Info("[Jump To App]\r\n");
        SrvUpgrade_Collect_Info("\tApp Address: 0x%08x\r\n", Monitor.JumpAddr);
        SrvUpgrade_Collect_Info("\tApp Size:    0x%08x\r\n", Monitor.AppSize);
        SrvOsCommon.delay_ms(10);

        /* disable all interrupt before jump to app */
        SrvOsCommon.disable_all_irq();

        jump_addr = *(volatile uint32_t *)(Monitor.JumpAddr + 4);
        __set_MSP(*(volatile uint32_t *)Monitor.JumpAddr);
        ((Application_Func)jump_addr)();
    }
}

static void SrvUpgrade_Collect_Info(const char *format, ...)
{
    va_list args;
    uint16_t buf_remain = 0;
    uint16_t buf_capacity = sizeof(Monitor.LogOut_Info);
    int16_t send_len = 0;

    if (format && (Monitor.LogOut_Info_size < buf_capacity))
    {
        buf_remain = buf_capacity - Monitor.LogOut_Info_size;

        va_start(args, format);

        send_len = vsnprintf((char *)(Monitor.LogOut_Info + Monitor.LogOut_Info_size), buf_remain, format, args);
        if (send_len > 0)
            Monitor.LogOut_Info_size += send_len;

        va_end(args);
    }
}

static uint16_t SrvUpgrade_Get_Info(uint8_t *p_info, uint16_t len)
{
    uint16_t log_size = 0;

    if (p_info && len && (len >= Monitor.LogOut_Info_size) && Monitor.init_state)
    {
        log_size = Monitor.LogOut_Info_size;
        memcpy(p_info, Monitor.LogOut_Info, Monitor.LogOut_Info_size);
        log_size = Monitor.LogOut_Info_size;
    }

    return log_size;
}

static void SrvUpgrade_ClearLog(void)
{
    memset(Monitor.LogOut_Info, 0, sizeof(Monitor.LogOut_Info));
    Monitor.LogOut_Info_size = 0;
}
