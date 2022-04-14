/*
 * globals.h
 *
 *  Created on: Dec 16, 2021
 *      Author: dato
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h"

#include "main.h"



typedef struct
{
	char buf[1024];
	uint8_t size;
}MEM_BUF;



typedef struct
{
	uint8_t local_ip[4];
	uint8_t server_ip[4];
	uint8_t gw[4];
	uint16_t local_port;
	uint16_t server_port;
}Network;


extern xSemaphoreHandle terminalMutex;
extern xSemaphoreHandle audioMutex;

extern MEM_BUF terminalbuf;
//extern osMutexId_t terminalMutexHandle;


RTC_TimeTypeDef currTime;
RTC_DateTypeDef currDate;
//extern volatile uint8_t loging;
extern uint8_t reboot;

#endif /* INC_GLOBALS_H_ */
