#include "drivers_buffer.h"

int Buffer::newBuffer( ptRingBuffer buffer, uint16_t size )
{
    if( buffer == NULL || size == 0 ) return -1;
    
    if(buffer->fifo == NULL)
    {
        buffer->fifo = (uint8_t*)malloc(size);
        if(buffer->fifo == NULL)    
        {
            return -1;
        }
    }
    buffer->pw = buffer->pr = 0;
    buffer->buf_size = size;
    
    return 0;
}

int Buffer::Buffer_Write( ptRingBuffer buffer, const uint8_t data )
{
    if(buffer == NULL || buffer->fifo==NULL)    return -1;
    int pt = (buffer->pw + 1) % buffer->buf_size;
    if(pt != buffer->pr)
    {
        buffer->fifo[buffer->pw] = data;
        buffer->pw = pt;
        
        return 0;
    }
    
    return -1;
}

int Buffer::Buffer_Write( ptRingBuffer buffer, const uint8_t *data_stream, uint8_t len )
{
    int pt = 0;
    if(buffer == NULL || buffer->fifo==NULL)    return -1;
    if(data_stream == NULL)     return -1;
    if(len == 0)    return -1;
    for(pt=0; pt<len; pt++)
    {
        if(Buffer_Write( buffer, data_stream[pt] ) != 0)    break;
    }
    
    return pt;
}

int Buffer::Buffer_Read( ptRingBuffer buffer, uint8_t *data )
{
    if(buffer == NULL || buffer->fifo==NULL)return -1;
    if(data == NULL)     return -1;
    if(buffer->pr == buffer->pw)return -1;
    
    *data = buffer->fifo[buffer->pr];
    buffer->pr = ( buffer->pr + 1 ) % buffer->buf_size;
    
    return 0;
}

int Buffer::Buffer_Read( ptRingBuffer buffer, uint8_t *data_stream, uint8_t len )
{
    int pt = 0;
    if(buffer == NULL || buffer->fifo==NULL)    return -1;
    if(data_stream == NULL)     return -1;
    if(len == 0)    return -1;
    for(pt=0; pt<len; pt++)
    {
        if( Buffer_Read( buffer, &data_stream[pt] ) != 0 )    break;
    }
    
    return pt;
}

int Buffer::Buffer_Clean( ptRingBuffer buffer )
{
    if(buffer == NULL || buffer->fifo==NULL)    return -1;
    memset(buffer->fifo, 0, buffer->buf_size);
    buffer->pw = buffer->pr = 0;
    
    return 0;
}
