/**
 * @file		uart_buf_g4.c
 * @brief		Hardware serial library for STM32G4, using hardware UART/USART
 * @author		Arturo Purizaga
 * @version		2.0a
 * @date		20-Oct-2021
 * @date		03-Dic-2021
 * MCU:			STM32G4 with built-in UART/USART
 * Compiler:	ARM GCC
 */

#include "uart_buf_g4.h"

/*
 * Constantes y macros
 */

/* Máscara para el buffer de recepción. */
#define LPUART1_RX_BUFFER_MASK	( LPUART1_RX_BUFFER_SIZE - 1 )
#define UART1_RX_BUFFER_MASK	( UART1_RX_BUFFER_SIZE - 1 )
#define UART2_RX_BUFFER_MASK	( UART2_RX_BUFFER_SIZE - 1 )
#define UART3_RX_BUFFER_MASK	( UART3_RX_BUFFER_SIZE - 1 )
#define UART4_RX_BUFFER_MASK	( UART4_RX_BUFFER_SIZE - 1 )
#define UART5_RX_BUFFER_MASK	( UART5_RX_BUFFER_SIZE - 1 )
/* Máscara para el buffer de transmisión. */
#define LPUART1_TX_BUFFER_MASK	( LPUART1_TX_BUFFER_SIZE - 1 )
#define UART1_TX_BUFFER_MASK	( UART1_TX_BUFFER_SIZE - 1 )
#define UART2_TX_BUFFER_MASK	( UART2_TX_BUFFER_SIZE - 1 )
#define UART3_TX_BUFFER_MASK	( UART3_TX_BUFFER_SIZE - 1 )
#define UART4_TX_BUFFER_MASK	( UART4_TX_BUFFER_SIZE - 1 )
#define UART5_TX_BUFFER_MASK	( UART5_TX_BUFFER_SIZE - 1 )

/* Comprobación de la potencia de 2 */
#if ( LPUART1_RX_BUFFER_SIZE & LPUART1_RX_BUFFER_MASK )
	#error "UART6 RX buffer size is not a power of 2"
#endif
#if ( LPUART1_TX_BUFFER_SIZE & LPUART1_TX_BUFFER_MASK )
	#error "UART6 TX buffer size is not a power of 2"
#endif

#if ( UART1_RX_BUFFER_SIZE & UART1_RX_BUFFER_MASK )
	#error "UART1 RX buffer size is not a power of 2"
#endif
#if ( UART1_TX_BUFFER_SIZE & UART1_TX_BUFFER_MASK )
	#error "UART1 TX buffer size is not a power of 2"
#endif

#if ( UART2_RX_BUFFER_SIZE & UART2_RX_BUFFER_MASK )
	#error "UART2 RX buffer size is not a power of 2"
#endif
#if ( UART2_TX_BUFFER_SIZE & UART2_TX_BUFFER_MASK )
	#error "UART2 TX buffer size is not a power of 2"
#endif

#if ( UART3_RX_BUFFER_SIZE & UART3_RX_BUFFER_MASK )
	#error "UART3 RX buffer size is not a power of 2"
#endif
#if ( UART3_TX_BUFFER_SIZE & UART3_TX_BUFFER_MASK )
	#error "UART3 TX buffer size is not a power of 2"
#endif

#if ( UART4_RX_BUFFER_SIZE & UART4_RX_BUFFER_MASK )
	#error "UART4 RX buffer size is not a power of 2"
#endif
#if ( UART4_TX_BUFFER_SIZE & UART4_TX_BUFFER_MASK )
	#error "UART4 TX buffer size is not a power of 2"
#endif

#if ( UART5_RX_BUFFER_SIZE & UART5_RX_BUFFER_MASK )
	#error "UART5 RX buffer size is not a power of 2"
#endif
#if ( UART5_TX_BUFFER_SIZE & UART5_TX_BUFFER_MASK )
	#error "UART5 TX buffer size is not a power of 2"
#endif


#if defined(LPUART1_ENABLED) && defined(LPUART1)
uart_buf_t lpuart1_buffer_handler;

static volatile unsigned char LPUART1_TxBuf[LPUART1_TX_BUFFER_SIZE];
static volatile unsigned char LPUART1_RxBuf[LPUART1_RX_BUFFER_SIZE];
#endif

#if defined(UART1_ENABLED) && defined(USART1)
uart_buf_t uart1_buffer_handler;
static volatile unsigned char UART1_TxBuf[UART1_TX_BUFFER_SIZE];
static volatile unsigned char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
#endif

#if defined(UART2_ENABLED) && defined(USART2)
uart_buf_t uart2_buffer_handler;

static volatile unsigned char UART2_TxBuf[UART2_TX_BUFFER_SIZE];
static volatile unsigned char UART2_RxBuf[UART2_RX_BUFFER_SIZE];
#endif

#if defined(UART3_ENABLED) && defined(USART3)
uart_buf_t uart3_buffer_handler;

static volatile unsigned char UART3_TxBuf[UART3_TX_BUFFER_SIZE];
static volatile unsigned char UART3_RxBuf[UART3_RX_BUFFER_SIZE];
#endif

#if defined(UART4_ENABLED) && defined(UART4)
uart_buf_t uart4_buffer_handler;

static volatile unsigned char UART4_TxBuf[UART4_TX_BUFFER_SIZE];
static volatile unsigned char UART4_RxBuf[UART4_RX_BUFFER_SIZE];
#endif

#if defined(UART5_ENABLED) && defined(UART5)
uart_buf_t uart5_buffer_handler;

static volatile unsigned char UART5_TxBuf[UART5_TX_BUFFER_SIZE];
static volatile unsigned char UART5_RxBuf[UART5_RX_BUFFER_SIZE];
#endif


void _read_config(uart_buf_init_t *param, uint8_t config)
{
	uint32_t databits = 0;
	uint32_t stopbits = 0;
	uint32_t parity = 0;

	switch (config & 0x07) {
	case 0x02:
		databits = 6;
		break;
	case 0x04:
		databits = 7;
		break;
	case 0x06:
		databits = 8;
		break;
	default:
		databits = 0;
		break;
	}

	if ((config & 0x30) == 0x30) {
		parity = UART_PARITY_ODD;
		databits++;
	} else if ((config & 0x20) == 0x20) {
		parity = UART_PARITY_EVEN;
		databits++;
	} else {
		parity = UART_PARITY_NONE;
	}

	if ((config & 0x08) == 0x08) {
		stopbits = UART_STOPBITS_2;
	} else {
		stopbits = UART_STOPBITS_1;
	}

	switch (databits) {
#ifdef UART_WORDLENGTH_7B
	case 7:
		databits = UART_WORDLENGTH_7B;
		break;
#endif
	case 8:
		databits = UART_WORDLENGTH_8B;
		break;
	case 9:
		databits = UART_WORDLENGTH_9B;
		break;
	default:
	case 0:
		return;
	}

	param->wordLength = databits;
	param->stopBits = stopbits;
	param->parity = parity;
	param->overSampling = UART_OVERSAMPLING_16;
}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
#if defined(LPUART1_ENABLED) && defined(LPUART1)
void _lpuart1_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == LPUART1) {
		// Enable GPIO TX/RX clock
		LPUART1_TX_GPIO_CLK_ENABLE();
#if LPUART1_TX_GPIO_CLK_ENABLE != LPUART1_RX_GPIO_CLK_ENABLE
		LPUART1_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		LPUART1_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = LPUART1_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = LPUART1_TX_AF;
		HAL_GPIO_Init(LPUART1_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = LPUART1_RX_PIN;
		GPIO_InitStruct.Alternate = LPUART1_RX_AF;
		HAL_GPIO_Init(LPUART1_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_LPUART1, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_LPUART1);
		HAL_NVIC_ClearPendingIRQ(IRQ_LPUART1);
	}
}
#endif

#if defined(UART1_ENABLED) && defined(USART1)
void _uart1_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == USART1) {
		// Enable GPIO TX/RX clock
		UART1_TX_GPIO_CLK_ENABLE();
#if UART1_TX_GPIO_CLK_ENABLE != UART1_RX_GPIO_CLK_ENABLE
		UART1_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		UART1_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = UART1_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = UART1_TX_AF;
		HAL_GPIO_Init(UART1_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = UART1_RX_PIN;
		GPIO_InitStruct.Alternate = UART1_RX_AF;
		HAL_GPIO_Init(UART1_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_UART1, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_UART1);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART1);
	}
}
#endif

#if defined(UART2_ENABLED) && defined(USART2)
void _uart2_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == USART2) {
		// Enable GPIO TX/RX clock
		UART2_TX_GPIO_CLK_ENABLE();
#if UART2_TX_GPIO_CLK_ENABLE != UART2_RX_GPIO_CLK_ENABLE
		UART2_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		UART2_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = UART2_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = UART2_TX_AF;
		HAL_GPIO_Init(UART2_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = UART2_RX_PIN;
		GPIO_InitStruct.Alternate = UART2_RX_AF;
		HAL_GPIO_Init(UART2_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_UART2, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_UART2);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART2);
	}
}
#endif

#if defined(UART3_ENABLED) && defined(USART3)
void _uart3_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == USART3) {
		// Enable GPIO TX/RX clock
		UART3_TX_GPIO_CLK_ENABLE();
#if UART3_TX_GPIO_CLK_ENABLE != UART3_RX_GPIO_CLK_ENABLE
		UART3_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		UART3_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = UART3_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = UART3_TX_AF;
		HAL_GPIO_Init(UART3_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = UART3_RX_PIN;
		GPIO_InitStruct.Alternate = UART3_RX_AF;
		HAL_GPIO_Init(UART3_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_UART3, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_UART3);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART3);
	}
}
#endif

#if defined(UART4_ENABLED) && defined(UART4)
void _uart4_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == UART4) {
		// Enable GPIO TX/RX clock
		UART4_TX_GPIO_CLK_ENABLE();
#if UART4_TX_GPIO_CLK_ENABLE != UART4_RX_GPIO_CLK_ENABLE
		UART4_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		UART4_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = UART4_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = UART4_TX_AF;
		HAL_GPIO_Init(UART4_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = UART4_RX_PIN;
		GPIO_InitStruct.Alternate = UART4_RX_AF;
		HAL_GPIO_Init(UART4_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_UART4, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_UART4);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART4);
	}
}
#endif

#if defined(UART5_ENABLED) && defined(UART5)
void _uart5_msp_init(USART_TypeDef *instance)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(instance == UART5) {
		// Enable GPIO TX/RX clock
		UART5_TX_GPIO_CLK_ENABLE();
#if UART5_TX_GPIO_CLK_ENABLE != UART5_RX_GPIO_CLK_ENABLE
		UART5_RX_GPIO_CLK_ENABLE();
#endif
		// Enable UARTX clock
		UART5_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = UART5_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = UART5_TX_AF;
		HAL_GPIO_Init(UART5_TX_GPIO_PORT, &GPIO_InitStruct);
		// UART RX GPIO pin configuration
		GPIO_InitStruct.Pin = UART5_RX_PIN;
		GPIO_InitStruct.Alternate = UART5_RX_AF;
		HAL_GPIO_Init(UART5_RX_GPIO_PORT, &GPIO_InitStruct);
		// NVIC for USART */
		HAL_NVIC_SetPriority(IRQ_UART5, 0, 1);
		HAL_NVIC_EnableIRQ(IRQ_UART5);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART5);
	}
}
#endif

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
#if defined(LPUART1_ENABLED) && defined(LPUART1)
void _lpuart1_msp_deinit(USART_TypeDef *instance)
{
	if(instance == LPUART1) {
		// Reset Peripherals
		LPUART1_FORCE_RESET();
		LPUART1_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(LPUART1_TX_GPIO_PORT, LPUART1_TX_PIN);
		HAL_GPIO_DeInit(LPUART1_RX_GPIO_PORT, LPUART1_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_LPUART1);
	}
}
#endif

#if defined(UART1_ENABLED) && defined(USART1)
void _uart1_msp_deinit(USART_TypeDef *instance)
{
	if(instance == USART1) {
		// Reset Peripherals
		UART1_FORCE_RESET();
		UART1_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART1_TX_GPIO_PORT, UART1_TX_PIN);
		HAL_GPIO_DeInit(UART1_RX_GPIO_PORT, UART1_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART1);
	}
}
#endif

#if defined(UART2_ENABLED) && defined(USART2)
void _uart2_msp_deinit(USART_TypeDef *instance)
{
	if(instance == USART2) {
		// Reset Peripherals
		UART2_FORCE_RESET();
		UART2_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART2_TX_GPIO_PORT, UART2_TX_PIN);
		HAL_GPIO_DeInit(UART2_RX_GPIO_PORT, UART2_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART2);
	}
}
#endif

#if defined(UART3_ENABLED) && defined(USART3)
void _uart3_msp_deinit(USART_TypeDef *instance)
{
	if(instance == USART3) {
		// Reset Peripherals
		UART3_FORCE_RESET();
		UART3_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART3_TX_GPIO_PORT, UART3_TX_PIN);
		HAL_GPIO_DeInit(UART3_RX_GPIO_PORT, UART3_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART3);
	}
}
#endif

#if defined(UART4_ENABLED) && defined(UART4)
void _uart4_msp_deinit(USART_TypeDef *instance)
{
	if(instance == UART4) {
		// Reset Peripherals
		UART4_FORCE_RESET();
		UART4_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART4_TX_GPIO_PORT, UART4_TX_PIN);
		HAL_GPIO_DeInit(UART4_RX_GPIO_PORT, UART4_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART4);
	}
}
#endif

#if defined(UART5_ENABLED) && defined(UART5)
void _uart5_msp_deinit(USART_TypeDef *instance)
{
	if(instance == UART5) {
		// Reset Peripherals
		UART5_FORCE_RESET();
		UART5_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART5_TX_GPIO_PORT, UART5_TX_PIN);
		HAL_GPIO_DeInit(UART5_RX_GPIO_PORT, UART5_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART5);
	}
}
#endif



#if defined(LPUART1_ENABLED) && defined(LPUART1)
bool LPUART1buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &lpuart1_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)LPUART1_TxBuf;
	param->tx_buffer_size = LPUART1_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)LPUART1_RxBuf;
	param->rx_buffer_size = LPUART1_RX_BUFFER_SIZE;
	param->options = options;

	lpuart1_buffer_handler._msp_init = _lpuart1_msp_init;
	lpuart1_buffer_handler._msp_deinit = _lpuart1_msp_deinit;

	// Initialize
	return uart_buf_init(&lpuart1_buffer_handler, LPUART1);
}

inline void LPUART1buf_end(void)
{
	uart_buf_end(&lpuart1_buffer_handler);
}

inline int LPUART1buf_getc(void)
{
	return uart_buf_getc(&lpuart1_buffer_handler);
}

inline int LPUART1buf_peek(void)
{
	return uart_buf_peek(&lpuart1_buffer_handler);
}

inline size_t LPUART1buf_putc(uint8_t data)
{
	return uart_buf_putc(&lpuart1_buffer_handler, data);
}

inline size_t LPUART1buf_puts(const char *s)
{
	return uart_buf_puts(&lpuart1_buffer_handler, s);
}

inline size_t LPUART1buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&lpuart1_buffer_handler, s, n);
}

inline int LPUART1buf_available(void)
{
	return uart_buf_available(&lpuart1_buffer_handler);
}

inline void LPUART1buf_flushRx(void)
{
	uart_buf_flushRx(&lpuart1_buffer_handler);
}

inline void LPUART1buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&lpuart1_buffer_handler, bDiscard);
}

void IRQHandler_LPUART1(void)
{
	uart_buf_irq_handler(&lpuart1_buffer_handler);
}
#endif

#if defined(UART1_ENABLED) && defined(USART1)
bool UART1buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &uart1_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)UART1_TxBuf;
	param->tx_buffer_size = UART1_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)UART1_RxBuf;
	param->rx_buffer_size = UART1_RX_BUFFER_SIZE;
	param->options = options;

	uart1_buffer_handler._msp_init = _uart1_msp_init;
	uart1_buffer_handler._msp_deinit = _uart1_msp_deinit;

	// Initialize
	return uart_buf_init(&uart1_buffer_handler, USART1);
}


inline void UART1buf_end(void)
{
	uart_buf_end(&uart1_buffer_handler);
}

inline int UART1buf_getc(void)
{
	return uart_buf_getc(&uart1_buffer_handler);
}

inline int UART1buf_peek(void)
{
	return uart_buf_peek(&uart1_buffer_handler);
}

inline size_t UART1buf_putc(uint8_t data)
{
	return uart_buf_putc(&uart1_buffer_handler, data);
}

inline size_t UART1buf_puts(const char *s)
{
	return uart_buf_puts(&uart1_buffer_handler, s);
}

inline size_t UART1buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&uart1_buffer_handler, s, n);
}

inline int UART1buf_available(void)
{
	return uart_buf_available(&uart1_buffer_handler);
}

inline void UART1buf_flushRx(void)
{
	uart_buf_flushRx(&uart1_buffer_handler);
}

inline void UART1buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&uart1_buffer_handler, bDiscard);
}

void IRQHandler_UART1(void)
{
	uart_buf_irq_handler(&uart1_buffer_handler);
}
#endif

#if defined(UART2_ENABLED) && defined(USART2)
bool UART2buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &uart2_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)UART2_TxBuf;
	param->tx_buffer_size = UART2_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)UART2_RxBuf;
	param->rx_buffer_size = UART2_RX_BUFFER_SIZE;
	param->options = options;

	uart2_buffer_handler._msp_init = _uart2_msp_init;
	uart2_buffer_handler._msp_deinit = _uart2_msp_deinit;

	// Initialize
	return uart_buf_init(&uart2_buffer_handler, USART2);
}

inline void UART2buf_end(void)
{
	uart_buf_end(&uart2_buffer_handler);
}

inline int UART2buf_getc(void)
{
	return uart_buf_getc(&uart2_buffer_handler);
}

inline int UART2buf_peek(void)
{
	return uart_buf_peek(&uart2_buffer_handler);
}

inline size_t UART2buf_putc(uint8_t data)
{
	return uart_buf_putc(&uart2_buffer_handler, data);
}

inline size_t UART2buf_puts(const char *s)
{
	return uart_buf_puts(&uart2_buffer_handler, s);
}

inline size_t UART2buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&uart2_buffer_handler, s, n);
}

inline int UART2buf_available(void)
{
	return uart_buf_available(&uart2_buffer_handler);
}

inline void UART2buf_flushRx(void)
{
	uart_buf_flushRx(&uart2_buffer_handler);
}

inline void UART2buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&uart2_buffer_handler, bDiscard);
}

void IRQHandler_UART2(void)
{
	uart_buf_irq_handler(&uart2_buffer_handler);
}
#endif

#if defined(UART3_ENABLED) && defined(USART3)
bool UART3buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &uart3_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)UART3_TxBuf;
	param->tx_buffer_size = UART3_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)UART3_RxBuf;
	param->rx_buffer_size = UART3_RX_BUFFER_SIZE;
	param->options = options;

	uart3_buffer_handler._msp_init = _uart3_msp_init;
	uart3_buffer_handler._msp_deinit = _uart3_msp_deinit;

	// Initialize
	return uart_buf_init(&uart3_buffer_handler, USART3);
}

inline void UART3buf_end(void)
{
	uart_buf_end(&uart3_buffer_handler);
}

inline int UART3buf_getc(void)
{
	return uart_buf_getc(&uart3_buffer_handler);
}

inline int UART3buf_peek(void)
{
	return uart_buf_peek(&uart3_buffer_handler);
}

inline size_t UART3buf_putc(uint8_t data)
{
	return uart_buf_putc(&uart3_buffer_handler, data);
}

inline size_t UART3buf_puts(const char *s)
{
	return uart_buf_puts(&uart3_buffer_handler, s);
}

inline size_t UART3buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&uart3_buffer_handler, s, n);
}

inline int UART3buf_available(void)
{
	return uart_buf_available(&uart3_buffer_handler);
}

inline void UART3buf_flushRx(void)
{
	uart_buf_flushRx(&uart3_buffer_handler);
}

inline void UART3buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&uart3_buffer_handler, bDiscard);
}

void IRQHandler_UART3(void)
{
	uart_buf_irq_handler(&uart3_buffer_handler);
}
#endif

#if defined(UART4_ENABLED) && defined(UART4)
bool UART4buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &uart4_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)UART4_TxBuf;
	param->tx_buffer_size = UART4_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)UART4_RxBuf;
	param->rx_buffer_size = UART4_RX_BUFFER_SIZE;
	param->options = options;

	uart4_buffer_handler._msp_init = _uart4_msp_init;
	uart4_buffer_handler._msp_deinit = _uart4_msp_deinit;

	// Initialize
	return uart_buf_init(&uart4_buffer_handler, UART4);
}

inline void UART4buf_end(void)
{
	uart_buf_end(&uart4_buffer_handler);
}

inline int UART4buf_getc(void)
{
	return uart_buf_getc(&uart4_buffer_handler);
}

inline int UART4buf_peek(void)
{
	return uart_buf_peek(&uart4_buffer_handler);
}

inline size_t UART4buf_putc(uint8_t data)
{
	return uart_buf_putc(&uart4_buffer_handler, data);
}

size_t UART4buf_puts(const char *s)
{
	return uart_buf_puts(&uart4_buffer_handler, s);
}

inline size_t UART4buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&uart4_buffer_handler, s, n);
}

inline int UART4buf_available(void)
{
	return uart_buf_available(&uart4_buffer_handler);
}

inline void UART4buf_flushRx(void)
{
	uart_buf_flushRx(&uart4_buffer_handler);
}

inline void UART4buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&uart4_buffer_handler, bDiscard);
}

void IRQHandler_UART4(void)
{
	uart_buf_irq_handler(&uart4_buffer_handler);
}
#endif

#if defined(UART5_ENABLED) && defined(UART5)
bool UART5buf_init(uint32_t baud, uint8_t config, uint8_t options)
{
	uart_buf_init_t *param = &uart5_buffer_handler.init;

	// Fill config parameters in struct
	_read_config(param, config);
	param->baudrate = baud;
	param->tx_buffer = (uint8_t *)UART5_TxBuf;
	param->tx_buffer_size = UART5_TX_BUFFER_SIZE;
	param->rx_buffer = (uint8_t *)UART5_RxBuf;
	param->rx_buffer_size = UART5_RX_BUFFER_SIZE;
	param->options = options;

	uart5_buffer_handler._msp_init = _uart5_msp_init;
	uart5_buffer_handler._msp_deinit = _uart5_msp_deinit;

	// Initialize
	return uart_buf_init(&uart5_buffer_handler, UART5);
}

inline void UART5buf_end(void)
{
	uart_buf_end(&uart5_buffer_handler);
}

inline int UART5buf_getc(void)
{
	return uart_buf_getc(&uart5_buffer_handler);
}

inline int UART5buf_peek(void)
{
	return uart_buf_peek(&uart5_buffer_handler);
}

inline size_t UART5buf_putc(uint8_t data)
{
	return uart_buf_putc(&uart5_buffer_handler, data);
}

inline size_t UART5buf_puts(const char *s)
{
	return uart_buf_puts(&uart5_buffer_handler, s);
}

inline size_t UART5buf_putn(const uint8_t *s, size_t n)
{
	return uart_buf_putn(&uart5_buffer_handler, s, n);
}

inline int UART5buf_available(void)
{
	return uart_buf_available(&uart5_buffer_handler);
}

inline void UART5buf_flushRx(void)
{
	uart_buf_flushRx(&uart5_buffer_handler);
}

inline void UART5buf_flushTx(bool bDiscard)
{
	uart_buf_flushTx(&uart5_buffer_handler, bDiscard);
}

void IRQHandler_UART5(void)
{
	uart_buf_irq_handler(&uart5_buffer_handler);
}
#endif
