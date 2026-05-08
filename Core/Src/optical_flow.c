/*
 * optical_flow.c
 *
 *  Created on: May 5, 2026
 *      Author: alex
 */

#include"optical_flow.h"
#include<string.h>

static UART_HandleTypeDef *_huart;
static uint8_t _rx_byte;
static uint8_t _buf[OF_PACKET_SIZE];
static uint8_t _buf_idx = 0;
static bool    _packet_ready = false;
static OpticalFlowData _latest;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len){
	uint16_t crc = 0xFFFF;
	for(uint16_t i = 0; i<len;i++){
		crc ^=data[i];
		for(uint8_t b= 0; b<8;b++){
			if(crc & 0x0001)
				crc = (crc >>1)^0xA001;
			else
				crc >>=1;
		}
	}
	return crc;
}

void OF_Init(UART_HandleTypeDef *huart){
	_huart = huart;
	_buf_idx = 0;
	_packet_ready = false;
	HAL_UART_Receive_IT(_huart, &_rx_byte, 1);
}

void OF_UartRXCpltCallback(void){
    uint8_t byte = _rx_byte;

    if(_buf_idx == 0 && byte != OF_START_BYTE){
        // просто пропускаем, не пишем в буфер
    } else {
        _buf[_buf_idx++] = byte;

        if(_buf_idx == OF_PACKET_SIZE){
            uint16_t crc_calc = crc16_modbus(_buf, 6);
            uint16_t crc_recv = (uint16_t)_buf[6] | ((uint16_t)_buf[7] << 8);

            if(crc_calc == crc_recv){
                _latest.px      = (int16_t)((uint16_t)_buf[1] | ((uint16_t)_buf[2] << 8));
                _latest.py      = (int16_t)((uint16_t)_buf[3] | ((uint16_t)_buf[4] << 8));
                _latest.quality = _buf[5];
                _latest.valid   = true;
                _packet_ready   = true;
            }
            _buf_idx = 0;
        }
    }

    HAL_UART_Receive_IT(_huart, &_rx_byte, 1); // всегда в конце
}
bool OF_GetData(OpticalFlowData *out){
	if(!_packet_ready) return false;
	__disable_irq();
	*out = _latest;
	_packet_ready = false;
	__enable_irq();
	return true;
}


