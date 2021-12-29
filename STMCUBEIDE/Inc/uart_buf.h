/**
 * @file		uart_buf.h
 * @brief		Hardware serial generic library using hardware UART/USART
 * @author		Arturo Purizaga
 * @version		2.0a
 * @date		19-Oct-2021
 * @date		22-Oct-2021
 * MCU:			STM32 MCU with built-in UART/USART
 * Compiler:	ARM GCC
 */

#ifndef UART_BUF_H_
#define UART_BUF_H_

#include <stdint.h>
#include <stdbool.h>

#if defined (STM32G431xx) || defined (STM32G441xx) || defined (STM32G471xx) || \
	defined (STM32G473xx) || defined (STM32G474xx) || defined (STM32G484xx) || \
	defined (STM32GBK1CB) || defined (STM32G491xx) || defined (STM32G4A1xx)
#include "stm32g4xx.h"
//#include "stm32g4xx_hal.h"
#define _UART_USE_G4
#elif defined (STM32F405xx) || defined (STM32F415xx) || defined (STM32F407xx) || defined (STM32F417xx) || \
	defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) || defined (STM32F439xx) || \
	defined (STM32F401xC) || defined (STM32F401xE) || defined (STM32F410Tx) || defined (STM32F410Cx) || \
	defined (STM32F410Rx) || defined (STM32F411xE) || defined (STM32F446xx) || defined (STM32F469xx) || \
	defined (STM32F479xx) || defined (STM32F412Cx) || defined (STM32F412Rx) || defined (STM32F412Vx) || \
	defined (STM32F412Zx) || defined (STM32F413xx) || defined (STM32F423xx)
#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"
#define _UART_USE_F4
#elif defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx) || defined (STM32F765xx) || \
	defined (STM32F767xx) || defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx) || \
	defined (STM32F722xx) || defined (STM32F723xx) || defined (STM32F732xx) || defined (STM32F733xx) || \
	defined (STM32F730xx) || defined (STM32F750xx)
#include "stm32f7xx.h"
//#include "stm32f7xx_hal.h"
#define _UART_USE_F7
#elif defined (STM32H743xx) || defined (STM32H753xx)  || defined (STM32H750xx) || defined (STM32H742xx) || \
	defined (STM32H745xx) || defined (STM32H755xx)  || defined (STM32H747xx) || defined (STM32H757xx) || \
	defined (STM32H7A3xx) || defined (STM32H7A3xxQ) || defined (STM32H7B3xx) || defined (STM32H7B3xxQ) || defined (STM32H7B0xx)  || defined (STM32H7B0xxQ) || \
	defined (STM32H735xx) || defined (STM32H733xx)  || defined (STM32H730xx) || defined (STM32H730xxQ)  || defined (STM32H725xx) || defined (STM32H723xx)
#include "stm32h7xx.h"
//#include "stm32h7xx_hal.h"
#define _UART_USE_H7
#elif defined (STM32L412xx) || defined (STM32L422xx) || \
	defined (STM32L431xx) || defined (STM32L432xx) || defined (STM32L433xx) || defined (STM32L442xx) || defined (STM32L443xx) || \
	defined (STM32L451xx) || defined (STM32L452xx) || defined (STM32L462xx) || \
	defined (STM32L471xx) || defined (STM32L475xx) || defined (STM32L476xx) || defined (STM32L485xx) || defined (STM32L486xx) || \
	defined (STM32L496xx) || defined (STM32L4A6xx) || \
	defined (STM32L4P5xx) || defined (STM32L4Q5xx) || \
	defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#include "stm32l4xx.h"
//#include "stm32l4xx_hal.h"
#define _UART_USE_L4
#else
// Just putting random stuffs if anything differente than others chips
#include "stm32f0xx.h"
//#include "stm32f4xx_hal.h"
#endif

#if !defined(HAL_UART_TIMEOUT_VALUE)
#define HAL_UART_TIMEOUT_VALUE              0x1FFFFFF
#endif

#include "ring_buffer.h"


typedef struct {
	uint32_t baudrate;		///< The baudrate
	uint32_t wordLength;	///< The length of data transmitted
	uint32_t parity;		///< Parity of data transmitted
	uint32_t stopBits;		///< The number of stop bits
	uint32_t overSampling;
	uint32_t options;
	uint8_t *tx_buffer;
	rb_size_t tx_buffer_size;
	uint8_t *rx_buffer;
	rb_size_t rx_buffer_size;
} uart_buf_init_t;


typedef struct {
	USART_TypeDef *instance;
	uart_buf_init_t init;
	void (*_msp_init)(USART_TypeDef *);
	void (*_msp_deinit)(USART_TypeDef *);
	volatile ring_buffer_t tx_buf;
	volatile ring_buffer_t rx_buf;
} uart_buf_t;


#define UART_BUF_RXTX_INVERT	0x01
#define UART_BUF_DATA_INVERT	0x02


bool uart_buf_init(uart_buf_t *handle, USART_TypeDef *uart);

void uart_buf_end(uart_buf_t *handle);

int uart_buf_getc(uart_buf_t *handle);

int uart_buf_peek(uart_buf_t *handle);

size_t uart_buf_putc(uart_buf_t *handle, uint8_t data);

size_t uart_buf_puts(uart_buf_t *handle, const char *s);

size_t uart_buf_putn(uart_buf_t *handle, const uint8_t *s, size_t n);

int uart_buf_available(uart_buf_t *handle);

void uart_buf_flushRx(uart_buf_t *handle);

void uart_buf_flushTx(uart_buf_t *handle, bool bDiscard);

void uart_buf_irq_handler(uart_buf_t *handle);

#endif /* UART_BUF_H_ */
