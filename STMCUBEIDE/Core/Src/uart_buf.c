/**
 * @file		uart_buf.c
 * @brief		Hardware serial generic library using hardware UART/USART
 * @author		Arturo Purizaga
 * @version		2.0a
 * @date		19-Oct-2021
 * @date		22-Oct-2021
 * MCU:			STM32 MCU with built-in UART/USART
 * Compiler:	ARM GCC
 */


#include "uart_buf.h"


#define USART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | \
                                      USART_CR1_OVER8 | USART_CR1_FIFOEN)) /*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */


#if defined(STM32H7)
#ifndef USART_ISR_RXNE
#define USART_ISR_RXNE	USART_ISR_RXNE_RXFNE
#endif
#ifndef USART_ISR_TXE
#define USART_ISR_TXE	USART_ISR_TXE_TXFNF
#endif
#endif

/*void _uart_setconfig(USART_TypeDef *uart, uart_buf_init_t *param)
{
	uint32_t tmpreg;

	// write CR1 configuration
	tmpreg = (uint32_t)param->wordLength | param->parity | UART_MODE_TX_RX | param->overSampling;
	MODIFY_REG(uart->CR1, USART_CR1_FIELDS, tmpreg);
	// write CR2 configuration
	MODIFY_REG(uart->CR2, USART_CR2_STOP, param->stopBits);
	// Write prescaler
	MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER, huart->Init.ClockPrescaler);
}*/

bool uart_buf_init(uart_buf_t *handle, USART_TypeDef *uart)
{
	UART_HandleTypeDef uartHandler = {0};

	// initialize ring buffers
	rb_init((ring_buffer_t *)&(handle->tx_buf), handle->init.tx_buffer_size, handle->init.tx_buffer);
	rb_init((ring_buffer_t *)&(handle->rx_buf), handle->init.rx_buffer_size, handle->init.rx_buffer);

	// Initialize uart
	handle->instance = uart;
	uartHandler.Instance        = uart;
	uartHandler.Init.BaudRate   = handle->init.baudrate;
	uartHandler.Init.WordLength = handle->init.wordLength;
	uartHandler.Init.StopBits   = handle->init.stopBits;
	uartHandler.Init.Parity     = handle->init.parity;
	uartHandler.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	uartHandler.Init.Mode       = UART_MODE_TX_RX;
	uartHandler.Init.OverSampling = handle->init.overSampling;
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx)\
	&& !defined(STM32L1xx)
	uartHandler.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (handle->init.options != 0) {
		if (handle->init.options & UART_BUF_RXTX_INVERT) {
			uartHandler.AdvancedInit.AdvFeatureInit |= (UART_ADVFEATURE_TXINVERT_INIT | UART_ADVFEATURE_RXINVERT_INIT);
			uartHandler.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
			uartHandler.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
		}
		if (handle->init.options & UART_BUF_DATA_INVERT) {
			uartHandler.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_DATAINVERT_INIT;
			uartHandler.AdvancedInit.DataInvert = UART_ADVFEATURE_DATAINV_ENABLE;
		}
	}

#endif
/*#ifdef UART_ONE_BIT_SAMPLE_DISABLE
	uartHandler->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif*/
	// DeInit USART before configuring
	uart_buf_end(handle);

	// Enable related gpio
	if (handle->_msp_init != NULL) {
		handle->_msp_init(uart);
	}

	// Init USART
	if(HAL_UART_Init(&uartHandler) != HAL_OK) {
		return false;
	}
//	// 1.- Disable UART
//	Instance->CR1 &= ~USART_CR1_UE;


	// Enable RX interrupt
	uart->CR1 |= USART_CR1_RXNEIE;

	return true;
}

void uart_buf_end(uart_buf_t *handle)
{
	USART_TypeDef *reg = handle->instance;

	// Disable peripheral
	reg->CR1 &= ~USART_CR1_UE;
	// Clear registers
	reg->CR1 = 0x0U;
	reg->CR2 = 0x0U;
	reg->CR3 = 0x0U;
	// Disable related gpio
	if (handle->_msp_deinit != NULL) {
		handle->_msp_deinit(reg);
	}
}

int uart_buf_getc(uart_buf_t *handle)
{
	return rb_safe_remove((ring_buffer_t *)&(handle->rx_buf));
}

int uart_buf_peek(uart_buf_t *handle)
{
	return rb_peek((ring_buffer_t *)&(handle->rx_buf));
}

size_t uart_buf_putc(uart_buf_t *handle, uint8_t data)
{
	USART_TypeDef *reg = handle->instance;

	// wait to having space in buffer
	while (rb_is_full((ring_buffer_t *)&(handle->tx_buf)));
	// write data
	rb_insert(&(handle->tx_buf), data);
	// enable tx interrupt
	reg->CR1 |= USART_CR1_TXEIE;

	return 1;
}

size_t uart_buf_puts(uart_buf_t *handle, const char *s)
{
	size_t numTransmit = 0;
	unsigned char temp;

	while ( (temp = *s++) ) {
		numTransmit += uart_buf_putc(handle, temp);
	}

	return numTransmit;
}

size_t uart_buf_putn(uart_buf_t *handle, const uint8_t *s, size_t n)
{
	size_t i;
	size_t numTransmit = 0;

	for(i = 0; i < n; i++) {
		numTransmit += uart_buf_putc(handle, *s++);
	}

	return numTransmit;
}

int uart_buf_available(uart_buf_t *handle)
{
	return (int)rb_full_count((ring_buffer_t *)&(handle->rx_buf));
}

void uart_buf_flushRx(uart_buf_t *handle)
{
	rb_reset((ring_buffer_t *)&(handle->rx_buf));
}

void uart_buf_flushTx(uart_buf_t *handle, bool bDiscard)
{
	USART_TypeDef *reg = handle->instance;

	if (bDiscard) {
		// disable tx interrupt
		reg->CR1 &= ~USART_CR1_TXEIE;
		// Clear the tx buffer
		rb_reset((ring_buffer_t *)&(handle->tx_buf));
		// enable tx interrupt
		reg->CR1 |= USART_CR1_TXEIE;
	} else {
		// Wait to empty the buffer
		while (!rb_is_empty((ring_buffer_t *)&(handle->tx_buf)));
		// Wait for complete transmision
		//while(reg->ISR & USART_ISR_TC);
		while((reg->ISR & USART_ISR_TC) == 0);
	}
}

void uart_buf_irq_handler(uart_buf_t *handle)
{
	USART_TypeDef *reg = handle->instance;
	uint32_t isrflags = READ_REG(reg->ISR);
	uint32_t cr1its = READ_REG(reg->CR1);
	//uint32_t cr3its = READ_REG(reg->CR3);

	uint32_t errorflags;
	uint8_t tmp;


	// Handling errors
	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_ORE));

	if (errorflags != 0) {	// Handle errors
		if(((isrflags & USART_ISR_PE) != 0) && ((cr1its & USART_CR1_PEIE) != 0)) {
			reg->ICR = USART_ICR_PECF;
		}
		if(((isrflags & USART_ISR_ORE) != 0) && ((cr1its & USART_CR1_RXNEIE) != 0)) {
			// Clear ORE flag
			reg->ICR = USART_ICR_ORECF;
		}
	}

	// UART reception mode
	if(((isrflags & USART_ISR_RXNE) != 0) && ((cr1its & USART_CR1_RXNEIE) != 0))
	{
		tmp = (uint8_t)(reg->RDR);

		rb_safe_insert((ring_buffer_t *)&(handle->rx_buf), tmp);
	}

	// UART transmision mode
	if(((isrflags & USART_ISR_TXE) != 0) && ((cr1its & USART_CR1_TXEIE) != 0))
	{
		if (rb_is_empty((ring_buffer_t *)&(handle->tx_buf))) {	// if nothing to send
			// disable tx interrupt
			reg->CR1 &= ~USART_CR1_TXEIE;
		} else {
			// read value from tx ring buffer
			tmp = rb_remove((ring_buffer_t *)&(handle->tx_buf));
			// and send
			reg->TDR = (uint32_t)tmp;
		}
	}
}

