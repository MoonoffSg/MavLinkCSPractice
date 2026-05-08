/*
 * optical_flow.h
 *
 *  Created on: May 5, 2026
 *      Author: alex
 */

#ifndef INC_OPTICAL_FLOW_H_
#define INC_OPTICAL_FLOW_H_
#include "stm32f0xx_hal.h"
#include<stdint.h>
#include<stdbool.h>

#define OF_PACKET_SIZE 8
#define OF_START_BYTE 0x55

typedef struct{
	int16_t px;
	int16_t py;
	int16_t quality;
	bool    valid;
}OpticalFlowData;

void OF_Init(UART_HandleTypeDef *huart);
bool OF_GetData(OpticalFlowData *out);
void OF_UartRXCpltCallback(void);


#endif /* INC_OPTICAL_FLOW_H_ */
