/*
 * lte.h
 *
 *  Created on: Jan 12, 2022
 *      Author: David Jashi
 */


/*
 *    request from server:
 *    	64 byte
 *
 *    	0 - 0
 *    	1 - cmd
 *    	2 - counter not really needed
 *    	4 - t0
 *    	8 - t1
 *    	12 - SETTINGS_SECTOR
 *    	16 - CLOCK_SECTOR
 *    	20 - DATA_SECTOR
 *    	24 - BAUD_RATE
 *
 *   response from sensor:
 *		72 byte
 *
 *
 */


#ifndef INC_LTE_H_
#define INC_LTE_H_

//#define BAUD_RATE 921600
#define BAUD_RATE 3200000
//#define BAUD_RATE 3686400

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
	uint8_t * const sram1;
	uint8_t * const sram2;
}LTE_PARAM;


void lteTaskRun(void* param);

#endif /* INC_LTE_H_ */
