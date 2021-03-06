/*
 * lte.h
 *
 *  Created on: Jan 12, 2022
 *      Author: David Jashi
 */

#ifndef INC_LTE_H_
#define INC_LTE_H_

#define BAUD_RATE 3200000

typedef enum
{

  LTE_BUSY = -2,
  LTE_ERROR = -1,
  LTE_NO    = 0,
  LTE_OK    = 1
}LTE_Status;

typedef struct
{
	UART_HandleTypeDef* huart;
}LTE_PARAM;


void lteTaskRun(void* param);

#endif /* INC_LTE_H_ */
