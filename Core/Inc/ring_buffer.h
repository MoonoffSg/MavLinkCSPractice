#ifndef __RING_BUFFER__
#define __RING_BUFFER__

#include<stdint.h>
#include<stdio.h>
#include<stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct RingBuf_struct
{
	uint8_t 	*pData;
	uint16_t 	size;
	uint16_t 	start;
	uint16_t 	end;
} RingBuf_t;

/**
  * @brief  initiate ring buffer.
  * @param  pRingBuf pointer to object structure.
  * @param  buf pointer to static buffer.
  * @param  size size of static buffer.
  * @retval operation status.
  */
bool ringBufInit(RingBuf_t *pRingBuf, uint8_t *pBuf, uint16_t size);
/**
  * @brief  put byte into buffer.
  * @param  pRingBuf pointer to object structure.
  * @param  ucByte data byte.
  * @retval operation status.
  */
bool ringBufPut(RingBuf_t *pRingBuf, uint8_t byte);
/**
  * @brief  put byte into buffer.
  * @param  pRingBuf pointer to object structure.
  * @param  byte poionter to data byte.
  * @retval operation status.
  */
bool ringBufGet(RingBuf_t* pRingBuf, uint8_t* byte);
/**
  * @brief  put byte array into buffer.
  * @param  pRingBuf pointer to object structure.
  * @param  data pointer to data byte array.
  * @param  size size of data.
  * @retval operation status.
  */
bool ringBufWrite(RingBuf_t *pRingBuf, uint8_t *data, uint16_t size);
/**
  * @brief  store byte array into buffer.
  * @param  pRingBuf pointer to object structure.
  * @param  data pointer to data byte array.
  * @param  size size of data.
  * @retval operation status.
  */
bool ringBufRead(RingBuf_t *pRingBuf, uint8_t *data, uint16_t size);
/**
  * @brief  get current buf size.
  * @param  pRingBuf pointer to object structure.
  * @retval size of buf.
  */
bool isringBufEmpty(RingBuf_t *pRingBuf);
uint16_t ringBufSize(RingBuf_t *pRingBuf);

#ifdef __cplusplus
}
#endif


#endif
