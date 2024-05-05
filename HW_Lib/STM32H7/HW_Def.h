#ifndef __HW_DEF_H
#define __HW_DEF_H

#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "Bsp_SPI.h"
#include "Bsp_SDMMC.h"
#include "Bsp_Uart.h"
#include "Bsp_IIC.h"
#include "Bsp_Flash.h"
#include "Dev_Led.h"

#define LED1_PIN GPIO_PIN_3
#define LED1_PORT GPIOE

#define LED2_PIN GPIO_PIN_4
#define LED2_PORT GPIOE

#define LED3_PIN GPIO_PIN_7
#define LED3_PORT GPIOH

/* SPI3 Reserve SPI */
#define RESERVE_SPI_CLK_PORT GPIOB
#define RESERVE_SPI_CLK_PIN GPIO_PIN_3

#define RESERVE_SPI_MISO_PORT GPIOB
#define RESERVE_SPI_MISO_PIN GPIO_PIN_4

#define RESERVE_SPI_MOSI_PORT GPIOB
#define RESERVE_SPI_MOSI_PIN GPIO_PIN_5

/* Baro IIC Pin */
#define BARO_BUS  BspIIC_Instance_I2C_2
#define IIC2_SDA_PORT GPIOB
#define IIC2_SDA_PIN GPIO_PIN_11
#define IIC2_SCK_PORT GPIOB
#define IIC2_SCK_PIN GPIO_PIN_10

/* Serial Pin */
#define UART4_TX_PORT GPIOB
#define UART4_TX_PIN GPIO_PIN_9
#define UART4_RX_PORT GPIOB
#define UART4_RX_PIN GPIO_PIN_8

#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN GPIO_PIN_9
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN GPIO_PIN_10
/* USB Detected Pin */
#define USB_DETECT_INT_PORT GPIOE
#define USB_DETECT_INT_PIN GPIO_PIN_2

/* PWM IO */
#define PWM_SIG_1_TIM TIM3
#define PWM_SIG_1_TIM_CHANNEL TIM_CHANNEL_3
#define PWM_SIG_1_PORT GPIOB
#define PWM_SIG_1_PIN GPIO_PIN_0
#define PWM_SIG_1_DMA Bsp_DMA_1
#define PWM_SIG_1_DMA_CHANNEL Bsp_DMA_Stream_0
#define PWM_SIG_1_PIN_AF GPIO_AF2_TIM3

#define PWM_SIG_2_TIM TIM3
#define PWM_SIG_2_TIM_CHANNEL TIM_CHANNEL_4
#define PWM_SIG_2_PORT GPIOB
#define PWM_SIG_2_PIN GPIO_PIN_1
#define PWM_SIG_2_DMA Bsp_DMA_1
#define PWM_SIG_2_DMA_CHANNEL Bsp_DMA_Stream_1
#define PWM_SIG_2_PIN_AF GPIO_AF2_TIM3

#define PWM_SIG_3_TIM TIM5
#define PWM_SIG_3_TIM_CHANNEL TIM_CHANNEL_1
#define PWM_SIG_3_PORT GPIOA
#define PWM_SIG_3_PIN GPIO_PIN_0
#define PWM_SIG_3_DMA Bsp_DMA_1
#define PWM_SIG_3_DMA_CHANNEL Bsp_DMA_Stream_2
#define PWM_SIG_3_PIN_AF GPIO_AF2_TIM5

#define PWM_SIG_4_TIM TIM5
#define PWM_SIG_4_TIM_CHANNEL TIM_CHANNEL_2
#define PWM_SIG_4_PORT GPIOA
#define PWM_SIG_4_PIN GPIO_PIN_1
#define PWM_SIG_4_DMA Bsp_DMA_1
#define PWM_SIG_4_DMA_CHANNEL Bsp_DMA_Stream_3
#define PWM_SIG_4_PIN_AF GPIO_AF2_TIM5

#define PWM_SIG_5_TIM TIM5
#define PWM_SIG_5_TIM_CHANNEL TIM_CHANNEL_3
#define PWM_SIG_5_PORT GPIOA
#define PWM_SIG_5_PIN GPIO_PIN_2
#define PWM_SIG_5_DMA Bsp_DMA_None
#define PWM_SIG_5_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_5_PIN_AF GPIO_AF2_TIM5

#define PWM_SIG_6_TIM TIM5
#define PWM_SIG_6_TIM_CHANNEL TIM_CHANNEL_4
#define PWM_SIG_6_PORT GPIOA
#define PWM_SIG_6_PIN GPIO_PIN_3
#define PWM_SIG_6_DMA Bsp_DMA_None
#define PWM_SIG_6_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_6_PIN_AF GPIO_AF2_TIM5

#define PWM_SIG_7_TIM TIM4
#define PWM_SIG_7_TIM_CHANNEL TIM_CHANNEL_1
#define PWM_SIG_7_PORT GPIOD
#define PWM_SIG_7_PIN GPIO_PIN_12
#define PWM_SIG_7_DMA Bsp_DMA_None
#define PWM_SIG_7_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_7_PIN_AF GPIO_AF2_TIM4

#define PWM_SIG_8_TIM TIM4
#define PWM_SIG_8_TIM_CHANNEL TIM_CHANNEL_2
#define PWM_SIG_8_PORT GPIOD
#define PWM_SIG_8_PIN GPIO_PIN_13
#define PWM_SIG_8_DMA Bsp_DMA_None
#define PWM_SIG_8_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_8_PIN_AF GPIO_AF2_TIM4

#define PWM_SIG_9_TIM TIM4
#define PWM_SIG_9_TIM_CHANNEL TIM_CHANNEL_3
#define PWM_SIG_9_PORT GPIOD
#define PWM_SIG_9_PIN GPIO_PIN_14
#define PWM_SIG_9_DMA Bsp_DMA_None
#define PWM_SIG_9_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_9_PIN_AF GPIO_AF2_TIM4

#define PWM_SIG_10_TIM TIM4
#define PWM_SIG_10_TIM_CHANNEL TIM_CHANNEL_4
#define PWM_SIG_10_PORT GPIOD
#define PWM_SIG_10_PIN GPIO_PIN_15
#define PWM_SIG_10_DMA Bsp_DMA_None
#define PWM_SIG_10_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_10_PIN_AF GPIO_AF2_TIM4

#define PWM_SIG_11_TIM TIM15
#define PWM_SIG_11_TIM_CHANNEL TIM_CHANNEL_1
#define PWM_SIG_11_PORT GPIOE
#define PWM_SIG_11_PIN GPIO_PIN_5
#define PWM_SIG_11_DMA Bsp_DMA_None
#define PWM_SIG_11_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_11_PIN_AF GPIO_AF2_TIM15

#define PWM_SIG_12_TIM TIM15
#define PWM_SIG_12_TIM_CHANNEL TIM_CHANNEL_2
#define PWM_SIG_12_PORT GPIOE
#define PWM_SIG_12_PIN GPIO_PIN_6
#define PWM_SIG_12_DMA Bsp_DMA_None
#define PWM_SIG_12_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_12_PIN_AF GPIO_AF2_TIM15

/* radio uart */
#define RADIO_PORT USART1

#define RADIO_TX_PIN UART1_TX_PIN
#define RADIO_RX_PIN UART1_RX_PIN

#define RADIO_TX_PIN_INIT_STATE false
#define RADIO_RX_PIN_INIT_STATE false

#define RADIO_TX_PIN_ALT GPIO_AF7_USART1
#define RADIO_RX_PIN_ALT GPIO_AF7_USART1

#define RADIO_TX_PORT UART1_TX_PORT
#define RADIO_RX_PORT UART1_RX_PORT

#define RADIO_TX_DMA Bsp_DMA_2
#define RADIO_TX_DMA_STREAM Bsp_DMA_Stream_0
#define RADIO_RX_DMA Bsp_DMA_2
#define RADIO_RX_DMA_STREAM Bsp_DMA_Stream_1

/* internal flash storage */
#define OnChipFlash_Storage_StartAddress (FLASH_BASE_ADDR + FLASH_SECTOR_7_OFFSET_ADDR)
#define OnChipFlash_Storage_TotalSize FLASH_SECTOR_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

#define OnChipFlash_MaxRWSize (2 Kb)
#define OnChipFlash_Storage_TabSize Flash_Storage_TabSize
#define OnChipFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize

/* external flash */
#define ExtFlash_Firmware_Addr 0
#define ExtFlash_Firmware_Size (1 Mb)

#define ExtFlash_Bus_Type Storage_ChipBus_None
#define ExtFlash_Chip_Type Storage_Chip_None
#define ExtFlash_Dev_Api NULL

#define ExtFlash_Start_Addr (ExtFlash_Firmware_Addr + ExtFlash_Firmware_Size)
#define ExtFlash_Storage_DefaultData FLASH_DEFAULT_DATA
#define ExtFlash_Storage_TotalSize (0 Kb)
#define ExtFlash_Storage_TabSize  Flash_Storage_TabSize
#define ExtFlash_Storage_InfoPageSize Flash_Storage_InfoPageSize

/* store boot info boot parameter and firmware */
#define ExternalFlash_BootDataSec_Size (0 Kb)
#define ExternalFlash_SysDataSec_Size (0 Kb)
#define ExternalFlash_UserDataSec_Size (0 Kb)

extern DevLedObj_TypeDef Led1;
extern DevLedObj_TypeDef Led2;
extern DevLedObj_TypeDef Led3;

extern BspGPIO_Obj_TypeDef USB_DctPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;
extern BspGPIO_Obj_TypeDef Uart1_TxPin;
extern BspGPIO_Obj_TypeDef Uart1_RxPin;



#endif
