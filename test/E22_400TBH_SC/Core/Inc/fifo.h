#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdint.h>

typedef enum
{
		FIFO_OK   = 0x00,
		FIFO_ERROR_NULL,
		FIFO_ERROR_LENGTH,
	  FIFO_ERROR_FULL,
	  FIFO_ERROR_EMPTY,
}fifo_error_t;

typedef struct
{
		uint32_t in;
		uint32_t out;
		uint32_t size;
		uint8_t* buffer;
} fifo_t;


fifo_error_t fifo_create( fifo_t *fifo , uint8_t *buffer, uint32_t size );
fifo_error_t fifo_clear( fifo_t *fifo );
fifo_error_t fifo_write( fifo_t *fifo, uint8_t *buffer, uint32_t length );
fifo_error_t fifo_read( fifo_t *fifo, uint8_t *buffer,  uint32_t length );
fifo_error_t fifo_get_length( fifo_t *fifo , uint32_t *length);
fifo_error_t fifo_get_remain_length( fifo_t *fifo , uint32_t *length);

#endif 
