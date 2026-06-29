#include "fifo.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))


fifo_error_t fifo_create( fifo_t *fifo , uint8_t *buffer, uint32_t size )
{
		/* 参数检查 */
		if( (fifo == 0) || (buffer == 0) || (size == 0) )
		{
			return FIFO_ERROR_NULL;
		}
		
		/* 长度检查 (必须是2^n) */
		if( size & (size - 1) )
		{
			return FIFO_ERROR_LENGTH;
		}
		
		/* 默认 */
		fifo->size = size;
		fifo->buffer = buffer;
		fifo->in = 0;
		fifo->out = 0;
		
		return FIFO_OK;
}

fifo_error_t fifo_clear( fifo_t *fifo )
{
		/* 参数检查 */
		if( fifo == 0 ) 
		{
			return FIFO_ERROR_NULL;
		}
		
    fifo->in = 0;
    fifo->out = 0;
	
    return FIFO_OK;
}


fifo_error_t fifo_write( fifo_t *fifo, uint8_t *buffer, uint32_t length )
{
    uint32_t i, j;
    uint32_t end_length, org_length;
    uint8_t *fifo_buffer;

		/* 记录原始用户写入长度 */
    org_length = length;
	
    /* 计算FIFO剩余容量 */
    length = MIN( length, fifo->size - fifo->in + fifo->out );
	
    /* 计算FIFO缓存区末尾是否还有足够位置 */
    end_length  = MIN( length, fifo->size - ( fifo->in & ( fifo->size - 1 ) ) );
	
		/* 找到FIFO可写入区域的起始地址 */
    fifo_buffer = fifo->buffer + ( fifo->in & ( fifo->size - 1 ) );
	
		/* 如果FIFO末尾有容量，则将用户数据复制到FIFO缓存区末尾 */
    for( i = 0; i < end_length; i++ )
    {
        *( fifo_buffer++ ) = *( buffer++ );
    }
    
		/* 计算用户数据是否已全部复制 */
    j = length - end_length;
		
		/* 如果用户数据有剩余，则需要从FIFO缓存区头部开始复制*/
    if ( j > 0 )
    {
        fifo_buffer = fifo->buffer;

        for( i = 0; i < j; i++ )
        {
            *( fifo_buffer++ ) = *( buffer++ );
        }
    }

		/* 记录已进入FIFO的长度 */
    fifo->in += length;
		
		/* 如果进入FIFO数据的长度小于用户实际写入数据的长度，那就是FIFO满了装不下了 */
    if( length < org_length )
    {
        return FIFO_ERROR_FULL;
    }
		
    return FIFO_OK;
}

fifo_error_t fifo_read( fifo_t *fifo, uint8_t *buffer,  uint32_t length )
{
    uint32_t i, j;
    uint32_t end_length, org_length;
    uint8_t *fifo_buffer;

		/* 记录原始用户读取长度 */
    org_length = length;
	
		/* 计算FIFO已入队数据长度 */
    length  = MIN( length, fifo->in - fifo->out );
	
    /* 计算FIFO末尾出队长度 */
    end_length = MIN( length, fifo->size - ( fifo->out & ( fifo->size - 1 ) ) );
	
		/* 找到FIFO末尾出队区域的起始地址 */
    fifo_buffer = fifo->buffer + ( fifo->out & ( fifo->size - 1 ) );
	
		/* 如果FIFO末尾有数据出队，则从FIFO内复制给用户数据缓存 */
    for( i = 0; i < end_length; i++ )
    {
        *( buffer++ ) = *( fifo_buffer++ );
    }
		
    /* 计算用户数据是否已全部复制 */
    j = length - end_length;
		
		/* 如果用户数据有剩余，则需要从FIFO缓存区头部开始复制*/
    if ( j > 0 )
    {
        fifo_buffer = fifo->buffer;

        for( i = 0; i < j; i++ )
        {
            *( buffer++ ) = *( fifo_buffer++ ) ;
        }
    }
		
		/* 记录已出队的长度 */
    fifo->out += length;
		
		/* 如果FIFO出队数据长度小于用户实际需要数据的长度，那就是FIFO空了数量不够 */
    if( length < org_length )
    {
        return FIFO_ERROR_EMPTY;
    }
		
    return FIFO_OK;
}

fifo_error_t fifo_get_length( fifo_t *fifo , uint32_t *length)
{
		if( (fifo==0) || (length==0) )
		{
				return FIFO_ERROR_NULL;
		}
		
		*length = fifo->in - fifo->out;
	
    return FIFO_OK;
}

fifo_error_t fifo_get_remain_length( fifo_t *fifo , uint32_t *length)
{
		if( (fifo==0) || (length==0) )
		{
				return FIFO_ERROR_NULL;
		}	
	
    *length = fifo->size - ( fifo->in - fifo->out );
		
		return FIFO_OK;
}


