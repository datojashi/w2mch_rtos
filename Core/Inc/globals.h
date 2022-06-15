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

enum  CMD
{
    cmd_ping_request    =   0x01U, // server <--> sensor
    cmd_ping_response   =   0x02U, // sensor <--> server

	cmd_AudioData_request   =   0x03U, // sensor --> server
	cmd_AudioData_response  =   0x04U, // server --> sensor

	cmd_startTransmit_request  =   0x05U, // server --> sensor
	cmd_startAudio_response =   0x06U, // sensor --> server

	cmd_stopTransmit_request   =   0x07U, //  server --> sensor
	cmd_stopTransmit_response  =   0x08U, //  sensor --> server

	//cmd_startLive_request   =   0x09U, // server --> sensor
    //cmd_startLive_response  =   0x0aU, // sensor --> server

	//cmd_stopLive_request   =   0x0bU, // server --> sensor
	//cmd_stopLive_response  =   0x0cU, // sensor --> server

	cmd_setConfig_request	= 0x0dU,	//server --> sensor
	cmd_setConfig_response	= 0x0eU,	//sensor --> server

    cmd_None    =0x00ffU
};

enum STATUS_FLAG{
	sf_Read	= 0x00U,
	sf_Send = 0x01U,
	sf_ReadError = 0x02,
	sf_No = 0x03,
	sf_Live = 0x04,
	sf_Error=0x06,
	sf_waitSettings=0x07
};

struct __attribute__((__packed__)) COMMAND
{
    uint8_t cmd;
    uint32_t start;
    uint32_t stop;
};

struct __attribute__((__packed__)) MESSAGE
{
    uint16_t tag;
    uint8_t nmb;
    uint8_t cmd;
    uint32_t sz;
};

struct __attribute__((__packed__)) CLOCK_DATA
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t mon;
	uint8_t year;

	uint32_t sector;
	uint16_t filler1;
	uint16_t filler2;
	uint16_t filler3;
};

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
extern xSemaphoreHandle rtcMutex;

extern MEM_BUF terminalbuf;
//extern osMutexId_t terminalMutexHandle;


extern RTC_TimeTypeDef currTime;
extern RTC_DateTypeDef currDate;
//extern volatile uint8_t loging;
extern uint8_t reboot;

#endif /* INC_GLOBALS_H_ */
