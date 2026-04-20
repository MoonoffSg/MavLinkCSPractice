#include <stddef.h>
#include "ring_buffer.h"

bool ringBufInit(RingBuf_t *pRingBuf, uint8_t *pBuf, uint16_t size)
{
    if (pRingBuf == NULL)
        return false;
    if (pBuf == NULL)
        return false;
    if (size == 0)
        return false;

    pRingBuf->pData = pBuf;
    pRingBuf->size = size;
    pRingBuf->start = 0;
    pRingBuf->end = 0;

    return true;
}

bool ringBufPut(RingBuf_t *pRingBuf, uint8_t byte)
{
    if (pRingBuf == NULL)
        return false;

    if (((pRingBuf->end + 1) % pRingBuf->size) != pRingBuf->start)
    {
        pRingBuf->pData[pRingBuf->end++] = byte;
        pRingBuf->end %= pRingBuf->size;
        return true;
    }

    return false;
}

bool ringBufGet(RingBuf_t *pRingBuf, uint8_t *pByte)
{
    if (pRingBuf == NULL)
        return false;

    if (pRingBuf->start != pRingBuf->end)
    {
        *pByte = pRingBuf->pData[pRingBuf->start++];
        pRingBuf->start %= pRingBuf->size;

        return true;
    }

    return false;
}

bool ringBufWrite(RingBuf_t *pRingBuf, uint8_t *pData, uint16_t size)
{
    while (size--)
    {
        if (!ringBufPut(pRingBuf, *pData++))
            return false;
    }

    return true;
}

bool ringBufRead(RingBuf_t *pRingBuf, uint8_t *pData, uint16_t size)
{
    while (size--)
    {
        if (!ringBufGet(pRingBuf, pData++))
            return false;
    }

    return true;
}

uint16_t ringBufSize(RingBuf_t *pRingBuf)
{
    if (pRingBuf == NULL)
        return 0;

    if (pRingBuf->end >= pRingBuf->start)
        return pRingBuf->end - pRingBuf->start;
    else
        return pRingBuf->size + pRingBuf->end - pRingBuf->start;
}
bool isringBufEmpty(RingBuf_t *pRingBuf){
	return (pRingBuf->start == pRingBuf->end);
}
