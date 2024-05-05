#include "HW_Def.h"

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led2 = {
    .port = LED2_PORT,
    .pin = LED2_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led3 = {
    .port = LED3_PORT,
    .pin = LED3_PIN,
    .init_state = true,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .pin = UART4_TX_PIN,
    .port = UART4_TX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .pin = UART4_RX_PIN,
    .port = UART4_RX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .pin = UART1_TX_PIN,
    .port = UART1_TX_PORT,
    .alternate = GPIO_AF7_USART1,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .pin = UART1_RX_PIN,
    .port = UART1_RX_PORT,
    .alternate = GPIO_AF7_USART1,
};

BspGPIO_Obj_TypeDef USB_DctPin = {
    .init_state = false,
    .pin = USB_DETECT_INT_PIN,
    .port = USB_DETECT_INT_PORT,
};
