/**
 * @file		ring_buffer.h
 * @brief		Simple implementation of a circular buffer
 * @author		Arturo Purizaga
 * @version		1.0a
 * @date		07-Jun-2016
 * @date		03-Dic-2021
 * MCU:			STM32 MCU
 * Compiler:	ARM GCC
 *
 * This implementation is not thread-safe. In particular, none of this functions
 * are guaranted reentrant.
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif



typedef uint16_t rb_size_t;

/**
 * @struct ring_buffer_t
 * @brief Ring buffer "type".
 *
 * Un byte es dejado libre para distinguir vac�o de lleno.
 *
 * @var ring_buffer_t::buf
 *  Pointer to the buffer
 * @var ring_buffer_t::tail
 *  Index of the component to read
 * @var ring_buffer_t::head
 *  Index of the component to be stored
 * @var ring_buffer_t::size
 *  Size of the buffer
 * @note
 *  Buffer is empty when head == tail.
 * @note
 *  Buffer is full when "tail" is a byte in front of "head", modulo buffer size.
 * @note
 *  A byte is left to differenciate "empty" from "full"
 *
 */
typedef struct RINGBUFFER {
	volatile uint8_t *buf;
	rb_size_t tail;			/**< �ndice al siguiente item a extraer. */
	rb_size_t head;			/**< �ndice al siguiente item a agregar. */
	rb_size_t size;			/**< Capacidad del buffer menos uno. */
} ring_buffer_t;

/**
 * @brief	Initialize circular buffer
 *
 * @param	rb		Instance
 * @param	size	Buffer size. Circular buffer always leave an
 * 					empty element so the maximum number of elements
 * 					are "size-1". Minimun size is 2.
 * @param	buf		Actual buffer
 */
static inline void rb_init(ring_buffer_t *rb, rb_size_t size, uint8_t *buf)
{
	rb->head = 0;
	rb->tail = 0;
	rb->size = size - 1;
	rb->buf = buf;
}

/**
 * @brief Return the number of elements stored in the buffer.
 * @param rb	ring buffer structure.
 */
static inline rb_size_t rb_full_count(ring_buffer_t *rb)
{
	volatile ring_buffer_t *arb = rb;
	int32_t size = arb->head - arb->tail;

	if (arb->head < arb->tail) {
		size += arb->size + 1;
	}

	return (rb_size_t)size;
}

/**
 * @brief Devuelve verdadero si y solo si el buffer est� lleno.
 * @param rb Buffer a testear.
 */
static inline bool rb_is_full(ring_buffer_t *rb)
{
	volatile ring_buffer_t *arb = rb;

	return (arb->head + 1 == arb->tail) || ((arb->head == arb->size) && (arb->tail == 0));
}

/**
 * @brief Devuelve verdadero si y solo si el buffer est� vac�o.
 * @param rb Buffer a testear.
 */
static inline bool rb_is_empty(ring_buffer_t *rb)
{
	volatile ring_buffer_t *arb = rb;

	return arb->tail == arb->head;
}

/**
 * A�ade un elemento al buffer circular.
 * @param rb Buffer donde se a�adir�.
 * @param element Valor a a�adir.
 */
static inline void rb_insert(ring_buffer_t *rb, uint8_t element)
{
	rb->buf[rb->head] = element;
	rb->head = (rb->head == rb->size) ? 0 : rb->head + 1;
}

/**
 * @brief Retira y devuelve el primer item del buffer circular.
 * @param rb Buffer de donde se remover�. Debe contener al menos un elemento.
 */
static inline uint8_t rb_remove(ring_buffer_t *rb)
{
	uint8_t ch = rb->buf[rb->tail];

	rb->tail = (rb->tail == rb->size) ? 0 : rb->tail + 1;
	return ch;
}

/**
 * @brief Devuelve el primer item del buffer circular, sin removerlo.
 * @param rb Buffer de donde se remover�. Debe contener al menos un elemento.
 */
static inline int rb_peek(ring_buffer_t *rb)
{
	if (rb->tail == rb->head) {
		return -1;
	} else {
		return (int)rb->buf[rb->tail];
	}
}

/**
 * @brief Intenta remover un �tem del buffer.
 *
 * Si el buffer no est� vac�o, remueve y retorna el �tem.
 * Si est� vac�o, no hace nada y retorna valor negativo.
 *
 * @param rb Buffer a usarse.
 */
static inline int rb_safe_remove(ring_buffer_t *rb)
{
	return rb_is_empty(rb) ? -1 : (int)rb_remove(rb);
}

/**
 * @brief Intenta insertar un elemento dentro del buffer.
 *
 * @param rb Buffer a usar.
 * @param element Valor a insertarse.
 * @sideeffect Si rb no esta lleno, agrega el elemento dentro de buffer.
 * @return Si el elemento es agregado, retorna verdadero; caso contrario, falso.
 */
static inline bool rb_safe_insert(ring_buffer_t *rb, uint8_t element)
{
	if (rb_is_full(rb)) {
		return false;
	}
	rb_insert(rb, element);
	return true;
}

/**
 * @brief Agrega un �tem al buffer circular no lleno.
 *
 * Si el buffer est� lleno, remueve el primer �tem, luego introduce el nuevo
 * elemento.
 *
 * @param rb Buffer a insertarse.
 * @param element Valor a insertarse en el buffer.
 * @return Retorna -1 cuando tiene �xito. Si un elemento fue retirado, retorna
 *         el valor retirado.
 */
static inline int rb_push_insert(ring_buffer_t *rb, uint8_t element)
{
	int ret = -1;

	if (rb_is_full(rb)) {
		ret = rb_remove(rb);
	}
	rb_insert(rb, element);
	return ret;
}

/**
 * @brief Descartar todos los �tems del buffer circular.
 * @param rb Buffer a limpiar.
 */
static inline void rb_reset(ring_buffer_t *rb)
{
	rb->head = rb->tail;
}

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* RING_BUFFER_H_ */
