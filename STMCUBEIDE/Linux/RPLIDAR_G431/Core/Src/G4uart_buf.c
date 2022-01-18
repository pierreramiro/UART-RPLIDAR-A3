/**
 * File:		uart_buf.h
 * Title:		Hardware serial library using ring buffer
 * Author:		Arturo Purizaga
 * MCU:			STM32F4 with built-in UART or USART (STM32F407 for now)
 * Compiler:	ARM GCC
 * Date:		16-May-2016
 * Brief:		This file provide firmware functions to control the UART or USART peripheral using a ring buffer.
 *
 */

//#include "stm32f4xx_hal_uart.h"
#include "G4uart_buf.h"
//#include "stm32f4xx_hal_uart.h"

/*
 * Constantes y macros
 */

/* M�scara para el buffer de recepci�n. */
#define UART1_RX_BUFFER_MASK ( UART1_RX_BUFFER_SIZE - 1 )
#define UART2_RX_BUFFER_MASK ( UART2_RX_BUFFER_SIZE - 1 )
#define UART3_RX_BUFFER_MASK ( UART3_RX_BUFFER_SIZE - 1 )
#define UART4_RX_BUFFER_MASK ( UART4_RX_BUFFER_SIZE - 1 )
#define UART5_RX_BUFFER_MASK ( UART5_RX_BUFFER_SIZE - 1 )
#define LPUART1_RX_BUFFER_MASK ( LPUART1_RX_BUFFER_SIZE - 1 )
/* M�scara para el buffer de transmisi�n. */
#define UART1_TX_BUFFER_MASK ( UART1_TX_BUFFER_SIZE - 1 )
#define UART2_TX_BUFFER_MASK ( UART2_TX_BUFFER_SIZE - 1 )
#define UART3_TX_BUFFER_MASK ( UART3_TX_BUFFER_SIZE - 1 )
#define UART4_TX_BUFFER_MASK ( UART4_TX_BUFFER_SIZE - 1 )
#define UART5_TX_BUFFER_MASK ( UART5_TX_BUFFER_SIZE - 1 )
#define LPUART1_TX_BUFFER_MASK ( LPUART1_TX_BUFFER_SIZE - 1 )

/* Comprobaci�n de la potencia de 2 */
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

#if ( LPUART1_RX_BUFFER_SIZE & LPUART1_RX_BUFFER_MASK )
	#error "LPUART1 RX buffer size is not a power of 2"
#endif
#if ( LPUART1_TX_BUFFER_SIZE & LPUART1_TX_BUFFER_MASK )
	#error "LPUART1 TX buffer size is not a power of 2"
#endif

#if defined(UART1_ENABLED)
UART_HandleTypeDef Uart1Handle;

static volatile uint32_t UART1_TxHead;
static volatile uint32_t UART1_TxTail;
static volatile unsigned char UART1_TxBuf[UART1_TX_BUFFER_SIZE];
static volatile uint32_t UART1_RxHead;
static volatile uint32_t UART1_RxTail;
static volatile unsigned char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
#endif
#if defined(UART2_ENABLED)
UART_HandleTypeDef Uart2Handle;

static volatile uint32_t UART2_TxHead;
static volatile uint32_t UART2_TxTail;
static volatile unsigned char UART2_TxBuf[UART2_TX_BUFFER_SIZE];
static volatile uint32_t UART2_RxHead;
static volatile uint32_t UART2_RxTail;
static volatile unsigned char UART2_RxBuf[UART2_RX_BUFFER_SIZE];
#endif
#if defined(UART3_ENABLED)
UART_HandleTypeDef Uart3Handle;

static volatile uint32_t UART3_TxHead;
static volatile uint32_t UART3_TxTail;
static volatile unsigned char UART3_TxBuf[UART3_TX_BUFFER_SIZE];
static volatile uint32_t UART3_RxHead;
static volatile uint32_t UART3_RxTail;
static volatile unsigned char UART3_RxBuf[UART3_RX_BUFFER_SIZE];
#endif
#if defined(UART4_ENABLED)
UART_HandleTypeDef Uart4Handle;

static volatile uint32_t UART4_TxHead;
static volatile uint32_t UART4_TxTail;
static volatile unsigned char UART4_TxBuf[UART4_TX_BUFFER_SIZE];
static volatile uint32_t UART4_RxHead;
static volatile uint32_t UART4_RxTail;
static volatile unsigned char UART4_RxBuf[UART4_RX_BUFFER_SIZE];
#endif
#if defined(UART5_ENABLED)
UART_HandleTypeDef Uart5Handle;

static volatile uint32_t UART5_TxHead;
static volatile uint32_t UART5_TxTail;
static volatile unsigned char UART5_TxBuf[UART5_TX_BUFFER_SIZE];
static volatile uint32_t UART5_RxHead;
static volatile uint32_t UART5_RxTail;
static volatile unsigned char UART5_RxBuf[UART5_RX_BUFFER_SIZE];
#endif
#if defined(LPUART1_ENABLED)
UART_HandleTypeDef Uart6Handle;

static volatile uint32_t LPUART1_TxHead;
static volatile uint32_t LPUART1_TxTail;
static volatile unsigned char LPUART1_TxBuf[LPUART1_TX_BUFFER_SIZE];
static volatile uint32_t LPUART1_RxHead;
static volatile uint32_t LPUART1_RxTail;
static volatile unsigned char LPUART1_RxBuf[LPUART1_RX_BUFFER_SIZE];
#endif


/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
#if defined(UART1_ENABLED)
	if(huart->Instance == USART1) {
		// Enable GPIO TX/RX clock
		UART1_TX_GPIO_CLK_ENABLE();
		UART1_RX_GPIO_CLK_ENABLE();
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
		// Clean Buffers
		UART1_TxHead = 0;
		UART1_TxTail = 0;
		UART1_RxHead = 0;
		UART1_RxTail = 0;
	}
#endif
#if defined(UART2_ENABLED)
	if(huart->Instance == USART2) {
		// Enable GPIO TX/RX clock
		UART2_TX_GPIO_CLK_ENABLE();
		UART2_RX_GPIO_CLK_ENABLE();
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
		// Clean Buffers
		UART2_TxHead = 0;
		UART2_TxTail = 0;
		UART2_RxHead = 0;
		UART2_RxTail = 0;
	}
#endif
#if defined(UART3_ENABLED)
	if(huart->Instance == USART3) {
		// Enable GPIO TX/RX clock
		UART3_TX_GPIO_CLK_ENABLE();
		UART3_RX_GPIO_CLK_ENABLE();
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
		// Clean Buffers
		UART3_TxHead = 0;
		UART3_TxTail = 0;
		UART3_RxHead = 0;
		UART3_RxTail = 0;
	}
#endif
#if defined(UART4_ENABLED)
	if(huart->Instance == UART4) {
		// Enable GPIO TX/RX clock
		UART4_TX_GPIO_CLK_ENABLE();
		UART4_RX_GPIO_CLK_ENABLE();
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
		HAL_NVIC_SetPriority(IRQ_UART4, 2, 0);
		HAL_NVIC_EnableIRQ(IRQ_UART4);
		HAL_NVIC_ClearPendingIRQ(IRQ_UART4);
		// Clean Buffers
		UART4_TxHead = 0;
		UART4_TxTail = 0;
		UART4_RxHead = 0;
		UART4_RxTail = 0;
	}
#endif
#if defined(UART5_ENABLED)
	if(huart->Instance == UART5) {
		// Enable GPIO TX/RX clock
		UART5_TX_GPIO_CLK_ENABLE();
		UART5_RX_GPIO_CLK_ENABLE();
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
		// Clean Buffers
		UART5_TxHead = 0;
		UART5_TxTail = 0;
		UART5_RxHead = 0;
		UART5_RxTail = 0;
	}
#endif
#if defined(LPUART1_ENABLED)
	if(huart->Instance == LPUART1) {
		// Enable GPIO TX/RX clock
		LPUART1_TX_GPIO_CLK_ENABLE();
		LPUART1_RX_GPIO_CLK_ENABLE();
		// Enable UARTX clock
		LPUART1_TX_GPIO_CLK_ENABLE();
		// UART TX GPIO pin configuration
		GPIO_InitStruct.Pin       = LPUART1_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
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
		// Clean Buffers
		LPUART1_TxHead = 0;
		LPUART1_TxTail = 0;
		LPUART1_RxHead = 0;
		LPUART1_RxTail = 0;
	}
#endif
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
#if defined(UART1_ENABLED)
	if(huart->Instance == USART1) {
		// Reset Peripherals
		UART1_FORCE_RESET();
		UART1_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART1_TX_GPIO_PORT, UART1_TX_PIN);
		HAL_GPIO_DeInit(UART1_RX_GPIO_PORT, UART1_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART1);
	}
#endif
#if defined(UART2_ENABLED)
	if(huart->Instance == USART2) {
		// Reset Peripherals
		UART2_FORCE_RESET();
		UART2_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART2_TX_GPIO_PORT, UART2_TX_PIN);
		HAL_GPIO_DeInit(UART2_RX_GPIO_PORT, UART2_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART2);
	}
#endif
#if defined(UART3_ENABLED)
	if(huart->Instance == USART3) {
		// Reset Peripherals
		UART3_FORCE_RESET();
		UART3_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART3_TX_GPIO_PORT, UART3_TX_PIN);
		HAL_GPIO_DeInit(UART3_RX_GPIO_PORT, UART3_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART3);
	}
#endif
#if defined(UART4_ENABLED)
	if(huart->Instance == UART4) {
		// Reset Peripherals
		UART4_FORCE_RESET();
		UART4_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART4_TX_GPIO_PORT, UART4_TX_PIN);
		HAL_GPIO_DeInit(UART4_RX_GPIO_PORT, UART4_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART4);
	}
#endif
#if defined(UART5_ENABLED)
	if(huart->Instance == UART5) {
		// Reset Peripherals
		UART5_FORCE_RESET();
		UART5_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(UART5_TX_GPIO_PORT, UART5_TX_PIN);
		HAL_GPIO_DeInit(UART5_RX_GPIO_PORT, UART5_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_UART5);
	}
#endif
#if defined(LPUART1_ENABLED)
	if(huart->Instance == LPUART1) {
		// Reset Peripherals
		LPUART1_FORCE_RESET();
		LPUART1_RELEASE_RESET();
		// Disable peripherals and GPIO clocks
		HAL_GPIO_DeInit(LPUART1_TX_GPIO_PORT, LPUART1_TX_PIN);
		HAL_GPIO_DeInit(LPUART1_RX_GPIO_PORT, LPUART1_RX_PIN);
		// Disable NVIC for UART
		HAL_NVIC_DisableIRQ(IRQ_LPUART1);
	}
#endif
}


#if defined(UART1_ENABLED)

void UART1buf_init(uint32_t baud)
{
	Uart1Handle.Instance        = USART1;
	Uart1Handle.Init.BaudRate   = baud;
	Uart1Handle.Init.WordLength = UART1_WORDLENGHT;
	Uart1Handle.Init.StopBits   = UART1_STOPBITS;
	Uart1Handle.Init.Parity     = UART1_PARITY;
	Uart1Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart1Handle.Init.Mode       = UART_MODE_TX_RX;
	Uart1Handle.Init.OverSampling = UART1_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&Uart1Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&Uart1Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&Uart1Handle, UART_IT_RXNE);
}

void UART1buf_end(void)
{
	if(HAL_UART_DeInit(&Uart1Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int UART1buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART1_RxTail;
	if (_tail == UART1_RxHead) {
		return -1;
	} else {
		data = UART1_RxBuf[_tail];
		UART1_RxTail = (_tail + 1) & UART1_RX_BUFFER_MASK;
		return data;
	}
}

int UART1buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART1_RxTail;
	if (_tail == UART1_RxHead) {
		return -1;
	}
	data = UART1_RxBuf[_tail];
	return data;
}

size_t UART1buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (UART1_TxHead + 1) & UART1_TX_BUFFER_MASK;

	while(_head == UART1_TxTail);

	UART1_TxBuf[UART1_TxHead] = data;
	UART1_TxHead = _head;
	__HAL_UART_ENABLE_IT(&Uart1Handle, UART_IT_TXE);

	return 1;
}

size_t UART1buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += UART1buf_putc(temp);
	}

	return numTransmit;
}

size_t UART1buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += UART1buf_putc(*s++);
	}

	return numTransmit;
}

int UART1buf_available(void)
{
	return((UART1_RxHead >= UART1_RxTail) ? (UART1_RxHead - UART1_RxTail) : UART1_RX_BUFFER_SIZE - (UART1_RxTail - UART1_RxHead));
	//return (UART1_RX_BUFFER_SIZE + UART1_RxHead - UART1_RxTail) % UART1_RX_BUFFER_SIZE;
}

void UART1buf_flushRx(void)
{
	UART1_RxHead = UART1_RxTail;
}

void UART1buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&Uart1Handle, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		UART1_TxHead = UART1_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&Uart1Handle, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (UART1_TxTail != UART1_TxHead);
		while(__HAL_UART_GET_FLAG(&Uart1Handle, UART_FLAG_TC) == RESET);
	}
}

void IRQHandler_UART1(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&Uart1Handle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart1Handle, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(Uart1Handle.Instance->RDR);

		_head = (UART1_RxHead + 1) & UART1_RX_BUFFER_MASK;
		if (_head != UART1_RxTail) {
			UART1_RxBuf[UART1_RxHead] = tmp;
			UART1_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart1Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&Uart1Handle, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart1Handle, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (UART1_TxHead == UART1_TxTail) {
			__HAL_UART_DISABLE_IT(&Uart1Handle, UART_IT_TXE);
		} else {
			_tail = UART1_TxTail;
			tmp = UART1_TxBuf[_tail];
			Uart1Handle.Instance->RDR = tmp;
			UART1_TxTail = (_tail + 1) & UART1_TX_BUFFER_MASK;
		}
	}
}
#endif

#if defined(UART2_ENABLED)

void UART2buf_init(uint32_t baud)
{
	Uart2Handle.Instance        = USART2;
	Uart2Handle.Init.BaudRate   = baud;
	Uart2Handle.Init.WordLength = UART2_WORDLENGHT;
	Uart2Handle.Init.StopBits   = UART2_STOPBITS;
	Uart2Handle.Init.Parity     = UART2_PARITY;
	Uart2Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart2Handle.Init.Mode       = UART_MODE_TX_RX;
	Uart2Handle.Init.OverSampling = UART2_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&Uart2Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&Uart2Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_RXNE);
}

void UART2buf_end(void)
{
	if(HAL_UART_DeInit(&Uart2Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int UART2buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART2_RxTail;
	if (_tail == UART2_RxHead) {
		return -1;
	} else {
		data = UART2_RxBuf[_tail];
		UART2_RxTail = (_tail + 1) & UART2_RX_BUFFER_MASK;
		return data;
	}
}

int UART2buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART2_RxTail;
	if (_tail == UART2_RxHead) {
		return -1;
	}
	data = UART2_RxBuf[_tail];
	return data;
}

size_t UART2buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (UART2_TxHead + 1) & UART2_TX_BUFFER_MASK;

	while(_head == UART2_TxTail);

	UART2_TxBuf[UART2_TxHead] = data;
	UART2_TxHead = _head;
	__HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_TXE);

	return 1;
}

size_t UART2buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += UART2buf_putc(temp);
	}

	return numTransmit;
}

size_t UART2buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += UART2buf_putc(*s++);
	}

	return numTransmit;
}

int UART2buf_available(void)
{
	return((UART2_RxHead >= UART2_RxTail) ? (UART2_RxHead - UART2_RxTail) : UART2_RX_BUFFER_SIZE - (UART2_RxTail - UART2_RxHead));
	//return (UART2_RX_BUFFER_SIZE + UART2_RxHead - UART2_RxTail) % UART2_RX_BUFFER_SIZE;
}

void UART2buf_flushRx(void)
{
	UART2_RxHead = UART2_RxTail;
}

void UART2buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&Uart2Handle, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		UART2_TxHead = UART2_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (UART2_TxTail != UART2_TxHead);
		while(__HAL_UART_GET_FLAG(&Uart2Handle, UART_FLAG_TC) == RESET);
	}
}

void USART2_IRQHandler(void)
//void IRQHandler_UART2(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&Uart2Handle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart2Handle, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(Uart2Handle.Instance->RDR);

		_head = (UART2_RxHead + 1) & UART2_RX_BUFFER_MASK;
		if (_head != UART2_RxTail) {
			UART2_RxBuf[UART2_RxHead] = tmp;
			UART2_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart2Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&Uart2Handle, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart2Handle, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (UART2_TxHead == UART2_TxTail) {
			__HAL_UART_DISABLE_IT(&Uart2Handle, UART_IT_TXE);
		} else {
			_tail = UART2_TxTail;
			tmp = UART2_TxBuf[_tail];
			Uart2Handle.Instance->TDR = tmp;
			UART2_TxTail = (_tail + 1) & UART2_TX_BUFFER_MASK;
		}
	}
}
#endif

#if defined(UART3_ENABLED)

void UART3buf_init(uint32_t baud)
{
	Uart3Handle.Instance        = USART3;
	Uart3Handle.Init.BaudRate   = baud;
	Uart3Handle.Init.WordLength = UART3_WORDLENGHT;
	Uart3Handle.Init.StopBits   = UART3_STOPBITS;
	Uart3Handle.Init.Parity     = UART3_PARITY;
	Uart3Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart3Handle.Init.Mode       = UART_MODE_TX_RX;
	Uart3Handle.Init.OverSampling = UART3_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&Uart3Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&Uart3Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&Uart3Handle, UART_IT_RXNE);
}

void UART3buf_end(void)
{
	if(HAL_UART_DeInit(&Uart3Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int UART3buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART3_RxTail;
	if (_tail == UART3_RxHead) {
		return -1;
	} else {
		data = UART3_RxBuf[_tail];
		UART3_RxTail = (_tail + 1) & UART3_RX_BUFFER_MASK;
		return data;
	}
}

int UART3buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART3_RxTail;
	if (_tail == UART3_RxHead) {
		return -1;
	}
	data = UART3_RxBuf[_tail];
	return data;
}

size_t UART3buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (UART3_TxHead + 1) & UART3_TX_BUFFER_MASK;

	while(_head == UART3_TxTail);

	UART3_TxBuf[UART3_TxHead] = data;
	UART3_TxHead = _head;
	__HAL_UART_ENABLE_IT(&Uart3Handle, UART_IT_TXE);

	return 1;
}

size_t UART3buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += UART3buf_putc(temp);
	}

	return numTransmit;
}

size_t UART3buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += UART3buf_putc(*s++);
	}

	return numTransmit;
}

int UART3buf_available(void)
{
	return((UART3_RxHead >= UART3_RxTail) ? (UART3_RxHead - UART3_RxTail) : UART3_RX_BUFFER_SIZE - (UART3_RxTail - UART3_RxHead));
	//return (UART3_RX_BUFFER_SIZE + UART3_RxHead - UART3_RxTail) % UART3_RX_BUFFER_SIZE;
}

void UART3buf_flushRx(void)
{
	UART3_RxHead = UART3_RxTail;
}

void UART3buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&Uart3Handle, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		UART3_TxHead = UART3_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&Uart3Handle, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (UART3_TxTail != UART3_TxHead);
		while(__HAL_UART_GET_FLAG(&Uart3Handle, UART_FLAG_TC) == RESET);
	}
}

void IRQHandler_UART3(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&Uart3Handle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart3Handle, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(Uart3Handle.Instance->RDR);

		_head = (UART3_RxHead + 1) & UART3_RX_BUFFER_MASK;
		if (_head != UART3_RxTail) {
			UART3_RxBuf[UART3_RxHead] = tmp;
			UART3_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart1Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&Uart3Handle, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart3Handle, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (UART3_TxHead == UART3_TxTail) {
			__HAL_UART_DISABLE_IT(&Uart3Handle, UART_IT_TXE);
		} else {
			_tail = UART3_TxTail;
			tmp = UART3_TxBuf[_tail];
			Uart3Handle.Instance->TDR = tmp;
			UART3_TxTail = (_tail + 1) & UART3_TX_BUFFER_MASK;
		}
	}
}
#endif

#if defined(UART4_ENABLED)

void UART4buf_init(uint32_t baud)
{
	Uart4Handle.Instance        = UART4;
	Uart4Handle.Init.BaudRate   = baud;
	Uart4Handle.Init.WordLength = UART4_WORDLENGHT;
	Uart4Handle.Init.StopBits   = UART4_STOPBITS;
	Uart4Handle.Init.Parity     = UART4_PARITY;
	Uart4Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart4Handle.Init.Mode       = UART_MODE_TX_RX;
	Uart4Handle.Init.OverSampling = UART4_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&Uart4Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&Uart4Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&Uart4Handle, UART_IT_RXNE);
}

void UART4buf_end(void)
{
	if(HAL_UART_DeInit(&Uart4Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int UART4buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART4_RxTail;
	if (_tail == UART4_RxHead) {
		return -1;
	} else {
		data = UART4_RxBuf[_tail];
		UART4_RxTail = (_tail + 1) & UART4_RX_BUFFER_MASK;
		return data;
	}
}

int UART4buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART4_RxTail;
	if (_tail == UART4_RxHead) {
		return -1;
	}
	data = UART4_RxBuf[_tail];
	return data;
}

size_t UART4buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (UART4_TxHead + 1) & UART4_TX_BUFFER_MASK;

	while(_head == UART4_TxTail);

	UART4_TxBuf[UART4_TxHead] = data;
	UART4_TxHead = _head;
	__HAL_UART_ENABLE_IT(&Uart4Handle, UART_IT_TXE);

	return 1;
}

size_t UART4buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += UART4buf_putc(temp);
	}

	return numTransmit;
}

size_t UART4buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += UART4buf_putc(*s++);
	}

	return numTransmit;
}

int UART4buf_available(void)
{
	return((UART4_RxHead >= UART4_RxTail) ? (UART4_RxHead - UART4_RxTail) : UART4_RX_BUFFER_SIZE - (UART4_RxTail - UART4_RxHead));
	//return (UART4_RX_BUFFER_SIZE + UART4_RxHead - UART4_RxTail) % UART4_RX_BUFFER_SIZE;
}

void UART4buf_flushRx(void)
{
	UART4_RxHead = UART4_RxTail;
}

void UART4buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&Uart4Handle, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		UART4_TxHead = UART4_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&Uart4Handle, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (UART4_TxTail != UART4_TxHead);
		while(__HAL_UART_GET_FLAG(&Uart4Handle, UART_FLAG_TC) == RESET);
	}
}

void IRQHandler_UART4(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&Uart4Handle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart4Handle, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(Uart4Handle.Instance->RDR);

		_head = (UART4_RxHead + 1) & UART4_RX_BUFFER_MASK;
		if (_head != UART4_RxTail) {
			UART4_RxBuf[UART4_RxHead] = tmp;
			UART4_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart4Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&Uart4Handle, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart4Handle, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (UART4_TxHead == UART4_TxTail) {
			__HAL_UART_DISABLE_IT(&Uart4Handle, UART_IT_TXE);
		} else {
			_tail = UART4_TxTail;
			tmp = UART4_TxBuf[_tail];
			Uart4Handle.Instance->TDR = tmp;
			UART4_TxTail = (_tail + 1) & UART4_TX_BUFFER_MASK;
		}
	}
}
#endif

#if defined(UART5_ENABLED)

void UART5buf_init(uint32_t baud)
{
	Uart5Handle.Instance        = UART5;
	Uart5Handle.Init.BaudRate   = baud;
	Uart5Handle.Init.WordLength = UART5_WORDLENGHT;
	Uart5Handle.Init.StopBits   = UART5_STOPBITS;
	Uart5Handle.Init.Parity     = UART5_PARITY;
	Uart5Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart5Handle.Init.Mode       = UART_MODE_TX_RX;
	Uart5Handle.Init.OverSampling = UART5_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&Uart5Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&Uart5Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&Uart5Handle, UART_IT_RXNE);
}

void UART5buf_end(void)
{
	if(HAL_UART_DeInit(&Uart5Handle) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int UART5buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART5_RxTail;
	if (_tail == UART5_RxHead) {
		return -1;
	} else {
		data = UART5_RxBuf[_tail];
		UART5_RxTail = (_tail + 1) & UART5_RX_BUFFER_MASK;
		return data;
	}
}

int UART5buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = UART5_RxTail;
	if (_tail == UART5_RxHead) {
		return -1;
	}
	data = UART5_RxBuf[_tail];
	return data;
}

size_t UART5buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (UART5_TxHead + 1) & UART5_TX_BUFFER_MASK;

	while(_head == UART5_TxTail);

	UART5_TxBuf[UART5_TxHead] = data;
	UART5_TxHead = _head;
	__HAL_UART_ENABLE_IT(&Uart5Handle, UART_IT_TXE);

	return 1;
}

size_t UART5buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += UART5buf_putc(temp);
	}

	return numTransmit;
}

size_t UART5buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += UART5buf_putc(*s++);
	}

	return numTransmit;
}

int UART5buf_available(void)
{
	return((UART5_RxHead >= UART5_RxTail) ? (UART5_RxHead - UART5_RxTail) : UART5_RX_BUFFER_SIZE - (UART5_RxTail - UART5_RxHead));
	//return (UART5_RX_BUFFER_SIZE + UART5_RxHead - UART5_RxTail) % UART5_RX_BUFFER_SIZE;
}

void UART5buf_flushRx(void)
{
	UART5_RxHead = UART5_RxTail;
}

void UART5buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&Uart5Handle, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		UART5_TxHead = UART5_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&Uart5Handle, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (UART5_TxTail != UART5_TxHead);
		while(__HAL_UART_GET_FLAG(&Uart5Handle, UART_FLAG_TC) == RESET);
	}
}

void IRQHandler_UART5(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&Uart5Handle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart5Handle, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(Uart5Handle.Instance->DR);

		_head = (UART5_RxHead + 1) & UART5_RX_BUFFER_MASK;
		if (_head != UART5_RxTail) {
			UART5_RxBuf[UART5_RxHead] = tmp;
			UART5_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart5Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&Uart5Handle, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&Uart5Handle, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (UART5_TxHead == UART5_TxTail) {
			__HAL_UART_DISABLE_IT(&Uart5Handle, UART_IT_TXE);
		} else {
			_tail = UART5_TxTail;
			tmp = UART5_TxBuf[_tail];
			Uart5Handle.Instance->DR = tmp;
			UART5_TxTail = (_tail + 1) & UART5_TX_BUFFER_MASK;
		}
	}
}
#endif

#if defined(LPUART1_ENABLED)

void LPUART1buf_init(uint32_t baud)
{
	hlpuart1.Instance        = LPUART1;
	hlpuart1.Init.BaudRate   = baud;
	hlpuart1.Init.WordLength = LPUART1_WORDLENGHT;
	hlpuart1.Init.StopBits   = LPUART1_STOPBITS;
	hlpuart1.Init.Parity     = LPUART1_PARITY;
	hlpuart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	hlpuart1.Init.Mode       = UART_MODE_TX_RX;
	hlpuart1.Init.OverSampling = LPUART1_OVERSAMPLING;
	// DeInit USART
	if(HAL_UART_DeInit(&hlpuart1) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Init USART
	if(HAL_UART_Init(&hlpuart1) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
	// Enable RX interrupt
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);
}

void LPUART1buf_end(void)
{
	if(HAL_UART_DeInit(&hlpuart1) != HAL_OK) {
		//Error_Handler();
		while(1);
	}
}

int LPUART1buf_getc(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = LPUART1_RxTail;
	if (_tail == LPUART1_RxHead) {
		return -1;
	} else {
		data = LPUART1_RxBuf[_tail];
		LPUART1_RxTail = (_tail + 1) & LPUART1_RX_BUFFER_MASK;
		return data;
	}
}

int LPUART1buf_peek(void)
{
	uint32_t _tail;
	uint8_t data;

	_tail = LPUART1_RxTail;
	if (_tail == LPUART1_RxHead) {
		return -1;
	}
	data = LPUART1_RxBuf[_tail];
	return data;
}

size_t LPUART1buf_putc(uint8_t data)
{
	uint32_t _head;

	_head = (LPUART1_TxHead + 1) & LPUART1_TX_BUFFER_MASK;

	while(_head == LPUART1_TxTail);

	LPUART1_TxBuf[LPUART1_TxHead] = data;
	LPUART1_TxHead = _head;
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TXE);

	return 1;
}

size_t LPUART1buf_puts(const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += LPUART1buf_putc(temp);
	}

	return numTransmit;
}

size_t LPUART1buf_putn(const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += LPUART1buf_putc(*s++);
	}

	return numTransmit;
}

int LPUART1buf_available(void)
{
	return((LPUART1_RxHead >= LPUART1_RxTail) ? (LPUART1_RxHead - LPUART1_RxTail) : LPUART1_RX_BUFFER_SIZE - (LPUART1_RxTail - LPUART1_RxHead));
	//return (UART0_RX_BUFFER_SIZE + UART0_RxHead - UART0_RxTail) % UART0_RX_BUFFER_SIZE;
}

void LPUART1buf_flushRx(void)
{
	LPUART1_RxHead = LPUART1_RxTail;
}

void LPUART1buf_flushTx(bool bDiscard)
{
	if (bDiscard) {
		// Desactiva interrupciones
		__HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_TXE);
		// Iguala punteros para "limpiar" el buffer
		LPUART1_TxHead = LPUART1_TxTail;
		// Habilitar interrupciones
		__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TXE);
	} else {
		// Espera a que termine la transmisi�n
		while (LPUART1_TxTail != LPUART1_TxHead);
		while(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET);
	}
}

void LPUART1_IRQHandler(void)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	uint32_t _head,_tail;
	uint8_t tmp;

	tmp1 = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_RXNE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		tmp = (uint8_t)(hlpuart1.Instance->RDR);

		_head = (LPUART1_RxHead + 1) & LPUART1_RX_BUFFER_MASK;
		if (_head != LPUART1_RxTail) {
			LPUART1_RxBuf[LPUART1_RxHead] = tmp;
			LPUART1_RxHead = _head;
		}
		//__HAL_UART_SEND_REQ(&Uart6Handle, UART_RXDATA_FLUSH_REQUEST);
	}

	tmp1 = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_TXE);

	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		if (LPUART1_TxHead == LPUART1_TxTail) {
			__HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_TXE);
		} else {
			_tail = LPUART1_TxTail;
			tmp = LPUART1_TxBuf[_tail];
			hlpuart1.Instance->TDR = tmp;
			LPUART1_TxTail = (_tail + 1) & LPUART1_TX_BUFFER_MASK;
		}
	}
}
#endif
