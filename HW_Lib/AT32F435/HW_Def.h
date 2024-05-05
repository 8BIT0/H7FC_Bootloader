#ifndef __HW_DEF_H
#define __HW_DEF_H

#include "bsp/Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "../../AT32F435/bsp/Bsp_Uart.h"
#include "Bsp_IIC.h"
#include "Bsp_SPI.h"
#include "Bsp_Flash.h"
#include "Dev_Led.h"
#include "util.h"

/* device support */
#include "Dev_W25Qxx.h"

/* radio port */
#define RADIO_PORT USART1
#define RADIO_TX_PIN_INIT_STATE GPIO_PULL_NONE
#define RADIO_RX_PIN_INIT_STATE GPIO_PULL_NONE
#define RADIO_TX_PIN_ALT GPIO_MUX_7
#define RADIO_RX_PIN_ALT GPIO_MUX_7
#define RADIO_TX_DMA Bsp_DMA_2
#define RADIO_TX_DMA_STREAM Bsp_DMA_Stream_1
#define RADIO_RX_DMA Bsp_DMA_2
#define RADIO_RX_DMA_STREAM Bsp_DMA_Stream_2
#define RADIO_TX_PIN GPIO_PINS_9
#define RADIO_RX_PIN GPIO_PINS_10
#define RADIO_TX_PORT &Uart1_Tx_Port
#define RADIO_RX_PORT &Uart1_Rx_Port

/* PWM IO */
#define PWM_SIG_1_TIM TMR3
#define PWM_SIG_1_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_1_PORT &PWM_1_Port
#define PWM_SIG_1_PIN GPIO_PINS_0
#define PWM_SIG_1_DMA Bsp_DMA_1
#define PWM_SIG_1_DMA_CHANNEL Bsp_DMA_Stream_1
#define PWM_SIG_1_PIN_AF GPIO_MUX_2

#define PWM_SIG_2_TIM TMR3
#define PWM_SIG_2_TIM_CHANNEL TMR_SELECT_CHANNEL_4
#define PWM_SIG_2_PORT &PWM_2_Port
#define PWM_SIG_2_PIN GPIO_PINS_1
#define PWM_SIG_2_DMA Bsp_DMA_1
#define PWM_SIG_2_DMA_CHANNEL Bsp_DMA_Stream_2
#define PWM_SIG_2_PIN_AF GPIO_MUX_2

#define PWM_SIG_3_TIM TMR2
#define PWM_SIG_3_TIM_CHANNEL TMR_SELECT_CHANNEL_4
#define PWM_SIG_3_PORT &PWM_3_Port
#define PWM_SIG_3_PIN GPIO_PINS_3
#define PWM_SIG_3_DMA Bsp_DMA_1
#define PWM_SIG_3_DMA_CHANNEL Bsp_DMA_Stream_3
#define PWM_SIG_3_PIN_AF GPIO_MUX_1

#define PWM_SIG_4_TIM TMR2
#define PWM_SIG_4_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_4_PORT &PWM_4_Port
#define PWM_SIG_4_PIN GPIO_PINS_2
#define PWM_SIG_4_DMA Bsp_DMA_1
#define PWM_SIG_4_DMA_CHANNEL Bsp_DMA_Stream_6
#define PWM_SIG_4_PIN_AF GPIO_MUX_1

#define PWM_SIG_5_TIM TMR8
#define PWM_SIG_5_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_5_PORT &PWM_5_Port
#define PWM_SIG_5_PIN GPIO_PINS_8
#define PWM_SIG_5_DMA Bsp_DMA_2
#define PWM_SIG_5_DMA_CHANNEL Bsp_DMA_Stream_1
#define PWM_SIG_5_PIN_AF GPIO_MUX_3

#define PWM_SIG_6_TIM TMR1
#define PWM_SIG_6_TIM_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_SIG_6_PORT &PWM_6_Port
#define PWM_SIG_6_PIN GPIO_PINS_8
#define PWM_SIG_6_DMA Bsp_DMA_2
#define PWM_SIG_6_DMA_CHANNEL Bsp_DMA_Stream_2
#define PWM_SIG_6_PIN_AF GPIO_MUX_1

/* led strip */
// #define PWM_SIG_7_TIM TMR4
// #define PWM_SIG_7_TIM_CHANNEL TMR_SELECT_CHANNEL_1
// #define PWM_SIG_7_PORT GPIOB
// #define PWM_SIG_7_PIN GPIO_PINS_6
// #define PWM_SIG_7_DMA Bsp_DMA_None
// #define PWM_SIG_7_DMA_CHANNEL Bsp_DMA_Stream_None
// #define PWM_SIG_7_PIN_AF GPIO_MUX_2

/* internal flash storage */
#define OnChipFlash_Storage_StartAddress FLASH_BLOCK_7_START_ADDR
#define OnChipFlash_Storage_TotalSize FLASH_BLOCK_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

/* external flash storage */
#if (FLASH_CHIP_STATE == ON)
#define ExtFlash_Bus_Type Storage_ChipBus_Spi
#define ExtFlash_Bus_Clock_Div SPI_MCLK_DIV_8
#define ExtFlash_Chip_Type Storage_ChipType_W25Qxx
#define ExtFlash_Bus_Api BspSPI
#define ExtFLash_Bus_Instance (void *)SPI2
#define ExtFlash_Bus_CLKPhase SPI_CLOCK_PHASE_2EDGE
#define ExtFlash_Bus_CLKPolarity SPI_CLOCK_POLARITY_HIGH
#define ExtFlash_CS_Pin ExtFlash_CSPin
#define ExtFlash_Bus_Pin ExtFlash_SPIPin

#define Boot_Firmware_Addr W25QXX_BASE_ADDRESS 
#define Boot_Firmware_Size (256 Kb)

#define Other_Firmware_Addr (Boot_Firmware_Addr + Boot_Firmware_Size)
#define Other_Firmware_Size (512 Kb)

#define Reserve_Addr (Other_Firmware_Addr + Other_Firmware_Size)
#define Reserve_Size ((1 Mb) - (Boot_Firmware_Size + Other_Firmware_Size))

#define ExtFlash_Firmware_Addr (Reserve_Addr + Reserve_Size)
#define ExtFlash_Firmware_Size (1 Mb)

#define ExtFlash_Dev_Api (void *)(&DevW25Qxx)
#define ExtFlash_Start_Addr (ExtFlash_Firmware_Addr + ExtFlash_Firmware_Size)

#define ExtFlash_Storage_DefaultData FLASH_DEFAULT_DATA
#define ExtFlash_Storage_TotalSize (384 Kb)
#define ExtFlash_Storage_TabSize  Flash_Storage_TabSize
#define ExtFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize

/* store boot info boot parameter and firmware */
#define ExternalFlash_BootDataSec_Size (32 Kb)
#define ExternalFlash_SysDataSec_Size (64 Kb)
#define ExternalFlash_UserDataSec_Size (64 Kb)

extern BspGPIO_Obj_TypeDef ExtFlash_CSPin;
extern BspSPI_PinConfig_TypeDef ExtFlash_SPIPin;
#endif

/* internal flash storage */
#define OnChipFlash_Storage_StartAddress FLASH_BLOCK_7_START_ADDR
#define OnChipFlash_Storage_TotalSize FLASH_BLOCK_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

#define OnChipFlash_MaxRWSize (2 Kb)
#define OnChipFlash_Storage_TabSize Flash_Storage_TabSize
#define OnChipFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize

extern BspGPIO_Port_TypeDef Uart3_Tx_Port;
extern BspGPIO_Port_TypeDef Uart3_Rx_Port;
extern BspGPIO_Port_TypeDef Uart1_Tx_Port;
extern BspGPIO_Port_TypeDef Uart1_Rx_Port;
extern BspGPIO_Port_TypeDef Uart4_Tx_Port;
extern BspGPIO_Port_TypeDef Uart5_Rx_Port;

extern BspGPIO_Obj_TypeDef Uart3_TxPin;
extern BspGPIO_Obj_TypeDef Uart3_RxPin;
extern BspGPIO_Obj_TypeDef Uart1_TxPin;
extern BspGPIO_Obj_TypeDef Uart1_RxPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;

extern DevLedObj_TypeDef Led1;

extern BspGPIO_Port_TypeDef PWM_1_Port;
extern BspGPIO_Port_TypeDef PWM_2_Port;
extern BspGPIO_Port_TypeDef PWM_3_Port;
extern BspGPIO_Port_TypeDef PWM_4_Port;
extern BspGPIO_Port_TypeDef PWM_5_Port;
extern BspGPIO_Port_TypeDef PWM_6_Port;

#endif
