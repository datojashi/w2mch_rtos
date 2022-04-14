/*
 * terminal.c
 *
 *  Created on: Dec 16, 2021
 *      Author: dato
 */

#include "main.h"
#include "globals.h"

#include "terminal.h"

extern RTC_HandleTypeDef hrtc;

TERMINAL_PARAM* terminal_param;

char ip[4];

uint8_t reboot=0;
Network netConfig;

void logConfig()
{
	uart_log("Local IP = ");
	for(int i=0; i<4; i++)
	{
		uart_log("%d",netConfig.local_ip[i]);
		if(i<3)
		{
			uart_log(".");
		}
	}
	uart_log("\r\n");
	uart_log("Server IP = ");
	for(int i=0; i<4; i++)
	{
		uart_log("%d",netConfig.server_ip[i]);
		if(i<3)
		{
			uart_log(".");
		}
	}
	uart_log("\r\n");
	uart_log("Gateway IP = ");
	for(int i=0; i<4; i++)
	{
		uart_log("%d",netConfig.gw[i]);
		if(i<3)
		{
			uart_log(".");
		}
	}
	uart_log("\r\n");
	uart_log("Local port = %d \r\n",netConfig.local_port);
	uart_log("Server Port = %d \r\n",netConfig.server_port);
}

void writeConfig()
{
	for(int i=0; i<4; i++)
	{
		HAL_RTCEx_BKUPWrite(&hrtc, i, netConfig.local_ip[i]);
		HAL_RTCEx_BKUPWrite(&hrtc, 4+i, netConfig.server_ip[i]);
		HAL_RTCEx_BKUPWrite(&hrtc, 8+i, netConfig.gw[i]);
	}
	HAL_RTCEx_BKUPWrite(&hrtc, 12, netConfig.local_port);
	HAL_RTCEx_BKUPWrite(&hrtc, 13, netConfig.server_port);
}

void readConfig()
{
	for(int i=0; i<4; i++)
	{
		netConfig.local_ip[i]=HAL_RTCEx_BKUPRead(&hrtc, i);
		netConfig.server_ip[i]=HAL_RTCEx_BKUPRead(&hrtc, 4+i);
		netConfig.gw[i]=HAL_RTCEx_BKUPRead(&hrtc, 8+i);
	}
	netConfig.local_port=HAL_RTCEx_BKUPRead(&hrtc, 12);
	netConfig.server_port=HAL_RTCEx_BKUPRead(&hrtc, 13);
	logConfig();
}

void parseIP(char* sip, uint8_t* ip)
{
	//LOG("* %s \r\n", sip);
	char delim[] = ".";
	char *ptr = strtok(sip, delim);
	int k=0;
	while(ptr != NULL && k < 4)
	{
		ip[k]=atoi(ptr);
		ptr = strtok(NULL, delim);
		k++;
	}
}

void parseCmd(char* buf)
{
	char cmd[8][16]={0};

	char delim[] = " ";
	char *ptr = strtok(buf, delim);

	 int k=0;
	 while(ptr != NULL)
	 {
	     sprintf(cmd[k],ptr);
	     ptr = strtok(NULL, delim);
	     k++;
	 }

	 /*
	 for(int i=0; i<k; i++)
	 {
		 LOG("--- %s \r\n",cmd[i]);
	 }
	 //*/

	 if(strcmp(cmd[0],"set")==0)
	 {
		 if(strcmp(cmd[1],"local")==0)
		 {
			 if(strcmp(cmd[2],"ip")==0)
			 {
				 parseIP(cmd[3], netConfig.local_ip);
			 }
			 else if(strcmp(cmd[2],"port")==0)
			 {
				 netConfig.local_port=atoi(cmd[3]);
			 }
		 }
		 else if(strcmp(cmd[1],"server")==0)
		 {
			 if(strcmp(cmd[2],"ip")==0)
			 {
				 parseIP(cmd[3], netConfig.server_ip);
			 }
			 else if(strcmp(cmd[2],"port")==0)
			 {
				 netConfig.server_port=atoi(cmd[3]);
			 }
		 }
		 else if(strcmp(cmd[1],"gw")==0)
		 {
			 parseIP(cmd[2], netConfig.gw);
		 }
		 writeConfig();
	 }
	 else if(strcmp(cmd[0],"read")==0)
	 {
		 readConfig();
	 }
	 else if(strcmp(cmd[0],"write")==0)
	 {
		 writeConfig();
	 }
	 else if(strcmp(cmd[0],"restart")==0)
	 {
		 writeConfig();
		 reboot=1;
	 }
	 else if(strcmp(cmd[0],"dt")==0)
	 {
		 HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
		 HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);
		 uart_log("%02d:%02d:%02d %02d.%02d.%02d\r\n",
				 currTime.Hours, currTime.Minutes, currTime.Seconds,
		 	     currDate.Date, currDate.Month, currDate.Year);
	 }
	 else if(strcmp(cmd[0],"")==0)
	 {

	 }
	 else
	 {
		 uart_log("Invalid command \"%s\" \r\n", cmd[0]);
	 }
}

void terminalRun(void* param)
{

	terminal_param = (TERMINAL_PARAM*)param;
	//readConfig();
	uint8_t sec=0;
	char buf[256]={0};
	//char p_prompt[]="w2mch>";
	//prompt=p_prompt;
	for(;;)
	{
		HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	    HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);
	    if(sec!=currTime.Seconds)
	    {
	    	//LOG("%02d:%02d:%02d %02d.%02d.%02d\r\n",currTime.Hours, currTime.Minutes, currTime.Seconds,
	        //                          currDate.Date, currDate.Month, currDate.Year);
		    sec=currTime.Seconds;
		}




	    xSemaphoreTake(terminalMutex,portMAX_DELAY);
	    if(terminalbuf.size==0)
	    {
	    	uart_log_prompt();
	    	terminalbuf.size=uart_recv(buf,128);
	    	memcpy(terminalbuf.buf,buf,128);
	    	//LOG("=== %d\r\n",sz);
	    }
	    xSemaphoreGive(terminalMutex);

	    if(strlen(buf)==0)
	    {
	    	loging=!loging;
	    }

	    memset(buf,0,256);
	    vTaskDelay(10);
	}
}
