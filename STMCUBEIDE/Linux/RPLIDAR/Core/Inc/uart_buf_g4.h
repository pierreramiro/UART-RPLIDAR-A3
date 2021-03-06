/**
 * @file		uart_buf_g4.h
 * @brief		Hardware serial library for STM32G4, using hardware UART/USART
 * @author		Arturo Purizaga
 * @version		2.0a
 * @date		20-Oct-2021
 * @date		03-Dic-2021
 * MCU:			STM32G4 with built-in UART/USART
 * Compiler:	ARM GCC
 */

/*
 * uart_buf_g4.h
 *
 *  Created on: Oct 20, 2021
 *      Author: Usuario
 */

#ifndef _UART_BUF_G4_H_
#define _UART_BUF_G4_H_

#include "uart_buf.h"

#if !defined(HAL_UART_TIMEOUT_VALUE)
#define HAL_UART_TIMEOUT_VALUE              0x1FFFFFF
#endif

/* Uncomment line below according to the UART/USART modules to use */

#define LPUART1_ENABLED
#define UART1_ENABLED
//#define UART2_ENABLED
//#define UART3_ENABLED
//#define UART4_ENABLED
//#define UART5_ENABLED


/** Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
#define LPUART1_RX_BUFFER_SIZE 64 /**< Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
#define UART1_RX_BUFFER_SIZE 1024 /**< Tamaño del buffer circular de recepci�n. Debe ser potencia de 2. */
#define UART2_RX_BUFFER_SIZE 128 /**< Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
#define UART3_RX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
#define UART4_RX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
#define UART5_RX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de recepci�n. Debe ser potencia de 2. */
/** Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define LPUART1_TX_BUFFER_SIZE 64 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define UART1_TX_BUFFER_SIZE 64 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define UART2_TX_BUFFER_SIZE 128 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define UART3_TX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define UART4_TX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */
#define UART5_TX_BUFFER_SIZE 32 /**< Tama�o del buffer circular de transmisi�n. Debe ser potencia de 2. */

/** Define configuraci�n del puerto serial */
// Used for

// below configs are not supported by STM32
//#define SERIAL_5N1 0x00
//#define SERIAL_5N2 0x08
//#define SERIAL_5E1 0x20
//#define SERIAL_5E2 0x28
//#define SERIAL_5O1 0x30
//#define SERIAL_5O2 0x38
//#define SERIAL_6N1 0x02
//#define SERIAL_6N2 0x0A

#ifdef UART_WORDLENGTH_7B
  #define SERIAL_6E1 0x22
  #define SERIAL_6E2 0x2A
  #define SERIAL_6O1 0x32
  #define SERIAL_6O2 0x3A

  #define SERIAL_7N1 0x04
  #define SERIAL_7N2 0x0C
#endif
#define SERIAL_7E1 0x24
#define SERIAL_7E2 0x2C
#define SERIAL_7O1 0x34
#define SERIAL_7O2 0x3C

#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E
#define SERIAL_8E1 0x26
#define SERIAL_8E2 0x2E
#define SERIAL_8O1 0x36
#define SERIAL_8O2 0x3E


/** Define ports to be used with UART/USART */
/** Pins used with LPUART1
 * 1: Pin PA2(TX) and PA3(RX)
 * 2: Pin PB11(TX) and PB10(RX)
 * 3: Pin PC1(TX) and PC0(RX)
 */
#define LPUART1_TX_PORT_SELECT	1
#define LPUART1_RX_PORT_SELECT	1

/** Pins used with USART1
 * 1: Pin PA9(TX) and PA10(RX)
 * 2: Pin PB6(TX) and PB7(RX)
 * 3: Pin PC4(TX) and PC5(RX)
 * 4: Pin PE0(TX) and PE1(RX)
 */
#define UART1_TX_PORT_SELECT	3
#define UART1_RX_PORT_SELECT	3

/** Pins used with USART2
 * 1: Pin PA2(TX) and PA3(RX)
 * 2: Pin PA14(TX) and PA15(RX)
 * 3: Pin PB3(TX) and PB4(RX)
 * 4: Pin PD5(TX) and PD6(RX)
 */
#define UART2_TX_PORT_SELECT	3
#define UART2_RX_PORT_SELECT	3

/** Pins used with USART3
 * 1: Pin PB10(TX) and PB11(RX)
 * 2: Pin PB9(TX) and PB8(RX)
 * 3: Pin PC10(TX) and PC11(RX)
 * 4: Pin PD8(TX) and PD9(RX)
 * 5: Pin PE15 (RX)
 */
#define UART3_TX_PORT_SELECT	2
#define UART3_RX_PORT_SELECT	2

/** Pins used with UART4
 * 1: Pin PC10(TX) and PC11(RX)
 */
#define UART4_TX_PORT_SELECT	1
#define UART4_RX_PORT_SELECT	1

/** Pins used with UART5
 * 1: Pin PC12(TX) and PD2(RX)
 */
#define UART5_TX_PORT_SELECT	1
#define UART5_RX_PORT_SELECT	1


/** Define resources for configuring UART/USART and gpi resourses*/
#if defined(LPUART1_ENABLED) && defined(LPUART1)

#define LPUART1_CLK_ENABLE()			__LPUART1_CLK_ENABLE()
#define LPUART1_FORCE_RESET()			__LPUART1_FORCE_RESET()
#define LPUART1_RELEASE_RESET()			__LPUART1_RELEASE_RESET()

#define IRQ_LPUART1						LPUART1_IRQn
#define IRQHandler_LPUART1				LPUART1_IRQHandler

#if LPUART1_TX_PORT_SELECT == 3
#define LPUART1_TX_PIN					GPIO_PIN_1
#define LPUART1_TX_GPIO_PORT			GPIOC
#define LPUART1_TX_AF					GPIO_AF8_LPUART1
#define LPUART1_TX_GPIO_CLK_ENABLE()	__GPIOC_CLK_ENABLE()
#elif LPUART1_TX_PORT_SELECT == 2
#define LPUART1_TX_PIN					GPIO_PIN_11
#define LPUART1_TX_GPIO_PORT			GPIOB
#define LPUART1_TX_AF					GPIO_AF8_LPUART1
#define LPUART1_TX_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#else
#define LPUART1_TX_PIN					GPIO_PIN_2
#define LPUART1_TX_GPIO_PORT			GPIOA
#define LPUART1_TX_AF					GPIO_AF12_LPUART1
#define LPUART1_TX_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#endif

#if LPUART1_RX_PORT_SELECT == 3
#define LPUART1_RX_PIN					GPIO_PIN_0
#define LPUART1_RX_GPIO_PORT			GPIOC
#define LPUART1_RX_AF					GPIO_AF8_LPUART1
#define LPUART1_RX_GPIO_CLK_ENABLE()	__GPIOC_CLK_ENABLE()
#elif LPUART1_RX_PORT_SELECT == 2
#define LPUART1_RX_PIN					GPIO_PIN_10
#define LPUART1_RX_GPIO_PORT			GPIOB
#define LPUART1_RX_AF					GPIO_AF8_LPUART1
#define LPUART1_RX_GPIO_CLK_ENABLE()	__GPIOB_CLK_ENABLE()
#else
#define LPUART1_RX_PIN					GPIO_PIN_3
#define LPUART1_RX_GPIO_PORT			GPIOA
#define LPUART1_RX_AF					GPIO_AF12_LPUART1
#define LPUART1_RX_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#endif

#endif

#if defined(UART1_ENABLED) && defined(USART1)

#define UART1_CLK_ENABLE()				__USART1_CLK_ENABLE()
#define UART1_FORCE_RESET()				__USART1_FORCE_RESET()
#define UART1_RELEASE_RESET()			__USART1_RELEASE_RESET()

#define IRQ_UART1						USART1_IRQn
#define IRQHandler_UART1				USART1_IRQHandler

#if UART1_TX_PORT_SELECT == 4
#define UART1_TX_PIN					GPIO_PIN_0
#define UART1_TX_GPIO_PORT				GPIOE
#define UART1_TX_AF						GPIO_AF7_USART1
#define UART1_TX_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#elif UART1_TX_PORT_SELECT == 3
#define UART1_TX_PIN					GPIO_PIN_4
#define UART1_TX_GPIO_PORT				GPIOC
#define UART1_TX_AF						GPIO_AF7_USART1
#define UART1_TX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#elif UART1_TX_PORT_SELECT == 2
#define UART1_TX_PIN					GPIO_PIN_6
#define UART1_TX_GPIO_PORT				GPIOB
#define UART1_TX_AF						GPIO_AF7_USART1
#define UART1_TX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#else
#define UART1_TX_PIN					GPIO_PIN_9
#define UART1_TX_GPIO_PORT				GPIOA
#define UART1_TX_AF						GPIO_AF7_USART1
#define UART1_TX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#endif

#if UART1_RX_PORT_SELECT == 4
#define UART1_RX_PIN					GPIO_PIN_1
#define UART1_RX_GPIO_PORT				GPIOE
#define UART1_RX_AF						GPIO_AF7_USART1
#define UART1_RX_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#elif UART1_RX_PORT_SELECT == 3
#define UART1_RX_PIN					GPIO_PIN_5
#define UART1_RX_GPIO_PORT				GPIOC
#define UART1_RX_AF						GPIO_AF7_USART1
#define UART1_RX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#elif UART1_RX_PORT_SELECT == 2
#define UART1_RX_PIN					GPIO_PIN_7
#define UART1_RX_GPIO_PORT				GPIOB
#define UART1_RX_AF						GPIO_AF7_USART1
#define UART1_RX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#else
#define UART1_RX_PIN					GPIO_PIN_10
#define UART1_RX_GPIO_PORT				GPIOA
#define UART1_RX_AF						GPIO_AF7_USART1
#define UART1_RX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#endif

#endif

#if defined(UART2_ENABLED) && defined(USART2)

#define UART2_CLK_ENABLE()				__USART2_CLK_ENABLE()
#define UART2_FORCE_RESET()				__USART2_FORCE_RESET()
#define UART2_RELEASE_RESET()			__USART2_RELEASE_RESET()

#define IRQ_UART2						USART2_IRQn
#define IRQHandler_UART2				USART2_IRQHandler

#if UART2_TX_PORT_SELECT == 4
#define UART2_TX_PIN					GPIO_PIN_5
#define UART2_TX_GPIO_PORT				GPIOD
#define UART2_TX_AF						GPIO_AF7_USART2
#define UART2_TX_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()
#elif UART2_TX_PORT_SELECT == 3
#define UART2_TX_PIN					GPIO_PIN_3
#define UART2_TX_GPIO_PORT				GPIOB
#define UART2_TX_AF						GPIO_AF7_USART2
#define UART2_TX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#elif UART2_TX_PORT_SELECT == 2
#define UART2_TX_PIN					GPIO_PIN_14
#define UART2_TX_GPIO_PORT				GPIOA
#define UART2_TX_AF						GPIO_AF7_USART2
#define UART2_TX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#else
#define UART2_TX_PIN					GPIO_PIN_2
#define UART2_TX_GPIO_PORT				GPIOA
#define UART2_TX_AF						GPIO_AF7_USART2
#define UART2_TX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#endif

#if UART2_RX_PORT_SELECT == 4
#define UART2_RX_PIN					GPIO_PIN_6
#define UART2_RX_GPIO_PORT				GPIOD
#define UART2_RX_AF						GPIO_AF7_USART2
#define UART2_RX_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()
#elif UART2_RX_PORT_SELECT == 3
#define UART2_RX_PIN					GPIO_PIN_4
#define UART2_RX_GPIO_PORT				GPIOB
#define UART2_RX_AF						GPIO_AF7_USART2
#define UART2_RX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#elif UART2_RX_PORT_SELECT == 2
#define UART2_RX_PIN					GPIO_PIN_15
#define UART2_RX_GPIO_PORT				GPIOA
#define UART2_RX_AF						GPIO_AF7_USART2
#define UART2_RX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#else
#define UART2_RX_PIN					GPIO_PIN_3
#define UART2_RX_GPIO_PORT				GPIOA
#define UART2_RX_AF						GPIO_AF7_USART2
#define UART2_RX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#endif

#endif

#if defined(UART3_ENABLED) && defined(USART3)

#define UART3_CLK_ENABLE()				__USART3_CLK_ENABLE()
#define UART3_FORCE_RESET()				__USART3_FORCE_RESET()
#define UART3_RELEASE_RESET()			__USART3_RELEASE_RESET()

#define IRQ_UART3						USART3_IRQn
#define IRQHandler_UART3				USART3_IRQHandler

#if UART3_TX_PORT_SELECT == 4
#define UART3_TX_PIN					GPIO_PIN_8
#define UART3_TX_GPIO_PORT				GPIOD
#define UART3_TX_AF						GPIO_AF7_USART3
#define UART3_TX_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()
#elif UART3_TX_PORT_SELECT == 3
#define UART3_TX_PIN					GPIO_PIN_10
#define UART3_TX_GPIO_PORT				GPIOC
#define UART3_TX_AF						GPIO_AF7_USART3
#define UART3_TX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#elif UART3_TX_PORT_SELECT == 2
#define UART3_TX_PIN					GPIO_PIN_10
#define UART3_TX_GPIO_PORT				GPIOB
#define UART3_TX_AF						GPIO_AF7_USART3
#define UART3_TX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#else
#define UART3_TX_PIN					GPIO_PIN_9
#define UART3_TX_GPIO_PORT				GPIOB
#define UART3_TX_AF						GPIO_AF7_USART3
#define UART3_TX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#endif

#if UART3_RX_PORT_SELECT == 5
#define UART3_RX_PIN					GPIO_PIN_15
#define UART3_RX_GPIO_PORT				GPIOE
#define UART3_RX_AF						GPIO_AF7_USART3
#define UART3_RX_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#elif UART3_RX_PORT_SELECT == 4
#define UART3_RX_PIN					GPIO_PIN_9
#define UART3_RX_GPIO_PORT				GPIOD
#define UART3_RX_AF						GPIO_AF7_USART3
#define UART3_RX_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()
#elif UART3_RX_PORT_SELECT == 3
#define UART3_RX_PIN					GPIO_PIN_11
#define UART3_RX_GPIO_PORT				GPIOC
#define UART3_RX_AF						GPIO_AF7_USART3
#define UART3_RX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#elif UART3_RX_PORT_SELECT == 2
#define UART3_RX_PIN					GPIO_PIN_11
#define UART3_RX_GPIO_PORT				GPIOB
#define UART3_RX_AF						GPIO_AF7_USART3
#define UART3_RX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#else
#define UART3_RX_PIN					GPIO_PIN_8
#define UART3_RX_GPIO_PORT				GPIOB
#define UART3_RX_AF						GPIO_AF7_USART3
#define UART3_RX_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#endif

#endif

#if defined(UART4_ENABLED) && defined(UART4)

#define UART4_CLK_ENABLE()				__UART4_CLK_ENABLE()
#define UART5_FORCE_RESET()				__UART4_FORCE_RESET()
#define UART5_RELEASE_RESET()			__UART4_RELEASE_RESET()

#define IRQ_UART4						UART4_IRQn
#define IRQHandler_UART4				UART4_IRQHandler

#define UART4_TX_PIN					GPIO_PIN_10
#define UART4_TX_GPIO_PORT				GPIOC
#define UART4_TX_AF						GPIO_AF5_UART4
#define UART4_TX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()

#define UART4_RX_PIN					GPIO_PIN_11
#define UART4_RX_GPIO_PORT				GPIOC
#define UART4_RX_AF						GPIO_AF5_UART4
#define UART4_RX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()

#endif

#if defined(UART5_ENABLED) && defined(UART5)

#define UART5_CLK_ENABLE()				__UART5_CLK_ENABLE()
#define UART5_FORCE_RESET()				__UART5_FORCE_RESET()
#define UART5_RELEASE_RESET()			__UART5_RELEASE_RESET()

#define IRQ_UART5						UART5_IRQn
#define IRQHandler_UART5				UART5_IRQHandler

#define UART5_TX_PIN					GPIO_PIN_12
#define UART5_TX_GPIO_PORT				GPIOC
#define UART5_TX_AF						GPIO_AF5_UART5
#define UART5_TX_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()

#define UART5_RX_PIN					GPIO_PIN_2
#define UART5_RX_GPIO_PORT				GPIOD
#define UART5_RX_AF						GPIO_AF5_UART5
#define UART5_RX_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()

#endif


/** Function Definitions */
#if defined(LPUART1_ENABLED) && defined(LPUART1)
bool LPUART1buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define LPUART1buf_clear_init(baud, config)	LPUART1buf_init((baud), (config), 0)
void LPUART1buf_end(void);
int LPUART1buf_getc(void);
int LPUART1buf_peek(void);
size_t LPUART1buf_putc(uint8_t data);
size_t LPUART1buf_puts(const char *s);
size_t LPUART1buf_putn(const uint8_t *s, size_t n);
int LPUART1buf_available(void);
void LPUART1buf_flushRx(void);
void LPUART1buf_flushTx(bool bDiscard);
#endif

#if defined(UART1_ENABLED) && defined(USART1)
bool UART1buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define UART1buf_clear_init(baud, config)	UART1buf_init((baud), (config), 0)
void UART1buf_end(void);
int UART1buf_getc(void);
int UART1buf_peek(void);
size_t UART1buf_putc(uint8_t data);
size_t UART1buf_puts(const char *s);
size_t UART1buf_putn(const uint8_t *s, size_t n);
int UART1buf_available(void);
void UART1buf_flushRx(void);
void UART1buf_flushTx(bool bDiscard);
#endif

#if defined(UART2_ENABLED) && defined(USART2)
bool UART2buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define UART2buf_clear_init(baud, config)	UART2buf_init((baud), (config), 0)
void UART2buf_end(void);
int UART2buf_getc(void);
int UART2buf_peek(void);
size_t UART2buf_putc(uint8_t data);
size_t UART2buf_puts(const char *s);
size_t UART2buf_putn(const uint8_t *s, size_t n);
int UART2buf_available(void);
void UART2buf_flushRx(void);
void UART2buf_flushTx(bool bDiscard);
#endif

#if defined(UART3_ENABLED) && defined(USART3)
bool UART3buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define UART3buf_clear_init(baud, config)	UART3buf_init((baud), (config), 0)
void UART3buf_end(void);
int UART3buf_getc(void);
int UART3buf_peek(void);
size_t UART3buf_putc(uint8_t data);
size_t UART3buf_puts(const char *s);
size_t UART3buf_putn(const uint8_t *s, size_t n);
int UART3buf_available(void);
void UART3buf_flushRx(void);
void UART3buf_flushTx(bool bDiscard);
#endif

#if defined(UART4_ENABLED) && defined(UART4)
bool UART4buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define UART4buf_clear_init(baud, config)	UART4buf_init((baud), (config), 0)
void UART4buf_end(void);
int UART4buf_getc(void);
int UART4buf_peek(void);
size_t UART4buf_putc(uint8_t data);
size_t UART4buf_puts(const char *s);
size_t UART4buf_putn(const uint8_t *s, size_t n);
int UART4buf_available(void);
void UART4buf_flushRx(void);
void UART4buf_flushTx(bool bDiscard);
#endif

#if defined(UART5_ENABLED) && defined(UART5)
bool UART5buf_init(uint32_t baud, uint8_t config, uint8_t options);
#define UART5buf_clear_init(baud, config)	UART5buf_init((baud), (config), 0)
void UART5buf_end(void);
int UART5buf_getc(void);
int UART5buf_peek(void);
size_t UART5buf_putc(uint8_t data);
size_t UART5buf_puts(const char *s);
size_t UART5buf_putn(const uint8_t *s, size_t n);
int UART5buf_available(void);
void UART5buf_flushRx(void);
void UART5buf_flushTx(bool bDiscard);
#endif

#endif /* _UART_BUF_G4_H_ */
