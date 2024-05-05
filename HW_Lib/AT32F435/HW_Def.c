#include "HW_Def.h"

/* Led 1 */
BspGPIO_Port_TypeDef Led1_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Led1 = {
    .port = (void *)&Led1_Port,
    .pin = GPIO_PINS_5,
    .init_state = GPIO_PULL_NONE,
};

/* on board receiver uart tx port */
BspGPIO_Port_TypeDef Uart3_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_TxPin = {
    .port = (void *)&Uart3_Tx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* on board receiver uart rx port */
BspGPIO_Port_TypeDef Uart3_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_RxPin = {
    .port = (void *)&Uart3_Rx_Port,
    .pin = GPIO_PINS_11,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 tx pin */
BspGPIO_Port_TypeDef Uart1_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .port = (void *)&Uart1_Tx_Port,
    .pin = GPIO_PINS_9,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 rx pin */
BspGPIO_Port_TypeDef Uart1_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .port = (void *)&Uart1_Rx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 4 tx pin */
BspGPIO_Port_TypeDef Uart4_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .port = (void *)&Uart4_Tx_Port,
    .pin = GPIO_PINS_0,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* uart 4 rx pin */
BspGPIO_Port_TypeDef Uart4_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .port = (void *)&Uart4_Rx_Port,
    .pin = GPIO_PINS_1,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_8,
};

/* external flash chip cs */
BspGPIO_Port_TypeDef ExtFlash_CS_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef ExtFlash_CSPin = {
    .port = (void *)&ExtFlash_CS_Port,
    .pin = GPIO_PINS_12,
    .init_state = GPIO_PULL_UP,
};

/* external flash chip bus Pin (SPI2) */
BspSPI_PinConfig_TypeDef ExtFlash_SPIPin = {
    .port_mosi = (void *)GPIOB,
    .port_miso = (void *)GPIOB,
    .port_clk = (void *)GPIOB,
    
    .pin_mosi = GPIO_PINS_15,
    .pin_miso = GPIO_PINS_14,
    .pin_clk = GPIO_PINS_13,

    .pin_Alternate = GPIO_MUX_5,
};

BspGPIO_Port_TypeDef PWM_1_Port = {
    .port = GPIOB,
};

BspGPIO_Port_TypeDef PWM_2_Port = {
    .port = GPIOB,
};

BspGPIO_Port_TypeDef PWM_3_Port = {
    .port = GPIOA,
};

BspGPIO_Port_TypeDef PWM_4_Port = {
    .port = GPIOA,
};

BspGPIO_Port_TypeDef PWM_5_Port = {
    .port = GPIOC,
};

BspGPIO_Port_TypeDef PWM_6_Port = {
    .port = GPIOA,
};
