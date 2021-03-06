/*
 * lte.c
 *
 *  Created on: Jan 12, 2022
 *      Author: David Jashi
 */


#include "globals.h"
#include "lte.h"



#define MAX_LTE_RESPONSE_LINES 8
#define MAX_LTE_RESPONSE_LINE_LENGTH 64
#define MAX_LTE_RESPONSE_RAW_LENGTH (MAX_LTE_RESPONSE_LINES*MAX_LTE_RESPONSE_LINE_LENGTH)



LTE_PARAM* lte_param;

char term_response[MAX_LTE_RESPONSE_LINES][MAX_LTE_RESPONSE_LINE_LENGTH];
char lte_response[MAX_LTE_RESPONSE_LINES][MAX_LTE_RESPONSE_LINE_LENGTH];
int lte_response_lines;
char lte_response_raw[MAX_LTE_RESPONSE_RAW_LENGTH];
volatile uint8_t lte_response_presence=0;
volatile uint8_t lte_response_size=0;

volatile uint8_t connected_to_server=0;

uint8_t lte_status=0;
uint8_t transparent=0;

char server_response[256];
volatile uint8_t recvcmplt=2;
volatile uint32_t recv_tickstart=0;

extern volatile uint8_t ready_for_send;
extern volatile uint16_t send_sz;
extern uint8_t* readbuf;
extern uint8_t* sendbuf;
extern uint8_t* sendcmd;


uint8_t pData[256];

void lteRecvHandler()
{
	HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//uart_log("** Cplt **\r\n");
	//HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//uart_log("** Half **\r\n");
	//HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);
	recvcmplt=1;
}

static inline  int lte_getLine(char* data)
{
	HAL_StatusTypeDef hstat=0;
	uint32_t	tickStart=0;
	uint8_t c=0, t=0;
	tickStart=HAL_GetTick();
	while(c!='\n' && t<256)
	{
		hstat = HAL_UART_Receive(lte_param->huart, &c, 1,100);
		if(hstat == HAL_OK && c!=0 &&  c!='\r' && c!='\n')
			data[t++]=c;

		if( (HAL_GetTick() - tickStart) > 10000)
		{
			return -2; //timeout
		}

	}
	data[t]=0;
	return t;
}


static inline  LTE_Status lte_getResp()
{
	int result=LTE_NO;
	//if(lte_response_presence==0)
	//	return result;
	int n=lte_response_size;
	if(n==0)
		return -2;
	int j=0,k=0;
	for(int i=0; i<n; i++)
	{
		char c=lte_response_raw[i];
		if(c=='\n')
		{
			k=0;
			j++;
		}
		else
		{
			if(c!='\r')
				lte_response[j][k++]=c;
		}
	}
	lte_response_lines=j;
	for(int i=0; i<lte_response_lines; i++)
	{
		if(strcmp(lte_response[i],"OK")==0)
		{
			result=LTE_OK;
			break;
		}
		else if(strcmp(lte_response[i],"ERROR")==0)
		{
			result=-1;
			break;
		}
		else if(strcmp(lte_response[i],"CONNECT 115200")==0)
		{
			result=1;
			break;
		}
		else if(strcmp(lte_response[i],"CONNECT 3200000")==0)
		{
			result=1;
			break;
		}
		char s[32];
		memset(s,0,32);
		sprintf(s,"CONNECT %d",BAUD_RATE);
		if(strcmp(lte_response[i],s)==0)
		{
			result=LTE_OK;
			break;
		}

	}
	return result;
}

static inline  void lte_response_log()
{
	for(int i=0; i<lte_response_lines; i++)
	{
		uart_log("\t%s \r\n",lte_response[i]);
	}
}


static inline void serverHandler()
{
	//if(server_response[0]==0 && server_response[1]==1)
	{
		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin,0);
	}
}

static inline HAL_StatusTypeDef lte_start_recv_DMA(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef hstat;
	recvcmplt=0;
	hstat = HAL_UART_Receive_DMA(huart, (uint8_t*)server_response, 256);
	if(hstat!=HAL_OK)
	{
		LOG("Error start receive DMA, for server commands\r\n");
	}
	return hstat;
}

static inline LTE_Status lte_receive_DMA(UART_HandleTypeDef *huart, uint32_t Timeout)
{
	uint32_t tickstart;
	int result=LTE_NO;
	HAL_StatusTypeDef hstat;

	hstat = HAL_UART_Receive_DMA(huart, (uint8_t*)lte_response_raw, MAX_LTE_RESPONSE_RAW_LENGTH);
	if(hstat==HAL_OK)
	{

		lte_response_size = 0;
		tickstart = HAL_GetTick();
		while ((lte_response_size < MAX_LTE_RESPONSE_RAW_LENGTH) && result!=1)
		{
			lte_response_size=MAX_LTE_RESPONSE_RAW_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			if( (HAL_GetTick() - tickstart) > Timeout)
			{
				uart_log("============ %s\r\n",lte_response_raw);
				uart_log("============ %d\r\n",lte_response_size);
				hstat=HAL_UART_AbortReceive(huart);
				return HAL_TIMEOUT;
			}
			result = lte_getResp();
		}
		//uart_log("---%u\r\n",__HAL_DMA_GET_COUNTER(huart->hdmarx));
		vTaskDelay(300);
		hstat=HAL_UART_AbortReceive(huart);
		if(hstat==HAL_OK)
			return result;
		else
			return LTE_ERROR;
	}
	else
	{
		uart_log("LTE Can't receive DMAS, hstat=%d\r\n",hstat);
		return LTE_ERROR;
	}



}

static inline LTE_Status lte_receive(UART_HandleTypeDef *huart,  uint32_t Timeout)
{
	uint32_t tickstart;
	uint8_t  *pdata;
	int result=LTE_NO;

	ATOMIC_SET_BIT(huart->Instance->RQR, USART_RQR_RXFRQ);
	ATOMIC_SET_BIT(huart->Instance->ICR, USART_ICR_ORECF);
	if (huart->RxState == HAL_UART_STATE_READY)
	{
	    pdata=(uint8_t*)lte_response_raw;
	    lte_response_size = 0;
	    tickstart = HAL_GetTick();

	    while ((lte_response_size < MAX_LTE_RESPONSE_RAW_LENGTH) && result!=1)
	    {
	    	if( (HAL_GetTick() - tickstart) > Timeout)
	    	{
	    		uart_log("============ %s\r\n",lte_response_raw);
	    		uart_log("============ %d\r\n",lte_response_size);
	    		return HAL_TIMEOUT;
	    	}
	    	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE)==0)
	    	{
	    		result = lte_getResp();
	    		continue;
	    	}
	        *pdata = (uint8_t)(huart->Instance->RDR & 0xffU);
	        pdata++;
	        lte_response_size++;
	        result = lte_getResp();
	    }
	    return result;
	}
	else
	{
		return LTE_BUSY;
	}

}

static inline  HAL_StatusTypeDef lte_send(char* data, size_t sz)
{
	HAL_StatusTypeDef hstat;
	hstat=HAL_UART_Transmit(lte_param->huart, (unsigned char*)data, sz, 100);
	if(hstat!=HAL_OK)
	{
		LOG("lte uart transmit error: %d \r\n",hstat);
	}
	return hstat;
	//LOG("lte_sendCmd %d \r\n",hstat);
}

static inline LTE_Status send_audio(char* data, int n)
{
	LTE_Status result=LTE_NO;
	char* d = data;

	for(int i=0; i<n; i++)
	{
		d=data+512*i;
		memcpy(sendbuf,d,512);
		sendcmd[2]=0; //channel
		sendcmd[3]=0x01; //cmd
		*((uint32_t*)&sendcmd[4])=512;
		if(lte_send((char*)sendcmd,560)==HAL_OK)
		{
			result=LTE_OK;
		}
		else
		{
			result=LTE_NO;
			LOG("Send error in transparent mode\r\n");
			break;
		}
		//vTaskDelay(1);
	}
	return result;
}

static inline LTE_Status processCommand(char* data, int n)
{
	LTE_Status result = LTE_NO;
	memset(lte_response_raw,0,MAX_LTE_RESPONSE_LINES*MAX_LTE_RESPONSE_LINE_LENGTH);
	for(int i=0; i<MAX_LTE_RESPONSE_LINES; i++)
				memset(lte_response[i],0,MAX_LTE_RESPONSE_LINE_LENGTH);
	lte_response_presence=0;
	//startReceive();
	lte_send(data,n);
	//result=lte_receive(lte_param->huart, 5000);
	result=lte_receive_DMA(lte_param->huart, 10000);


	//uart_log("--- %s   %d\r\n",lte_response_raw,lte_response_size);

	if(result < 0)
	{
		uart_log("Receive error %d\r\n", result);
	}
	lte_response_log();
	return result;
}




static inline LTE_Status lte_at_ccid()
{
	char data[64];
	size_t n=sprintf(data,"AT+CCID\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_check_transparent()
{
	char data[64];
	size_t n=sprintf(data,"AT+CIPMODE?\r\n");
	LTE_Status result=processCommand(data, n);
	if(result==LTE_OK)
	{
		if(strrchr(lte_response[1],'0')!=NULL)
		{
			result=LTE_NO;
		}
	}
	return result;
}


static inline LTE_Status lte_transparent()
{
	char data[64];
	size_t n=sprintf(data,"AT+CIPMODE=1\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_netopen()
{
	char data[64];
	size_t n=sprintf(data,"AT+NETOPEN\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_check_net()
{
	char data[64];
	size_t n=sprintf(data,"AT+NETOPEN?\r\n");
	LTE_Status result=processCommand(data, n);
	if(result==LTE_OK)
	{
		if(strrchr(lte_response[1],'0')!=NULL)
		{
			result=LTE_NO;
		}
	}
	return result;
}

static inline LTE_Status lte_tcp_open()
{
	char data[64];
	size_t n=sprintf(data,"AT+CIPOPEN=0,\"TCP\",\"185.51.101.235\",1234\r\n");
	LOG("%s",data);
	return processCommand(data, n);
}



static inline LTE_Status lte_ipaddr()
{
	char data[64];
	size_t n=sprintf(data,"AT+IPADDR\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_netclose()
{
	char data[64];
	size_t n=sprintf(data,"AT+NETCLOSE\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_endtransparent()
{
	char data[64];
	size_t n=sprintf(data,"+++");
	return processCommand(data, n);
}

static inline LTE_Status lte_tcp_close()
{
	char data[64];
	size_t n=sprintf(data,"AT+CIPCLOSE=0\r\n");
	return processCommand(data, n);
}

static inline LTE_Status lte_tcp_check()
{
	char data[64];
	size_t n=sprintf(data,"AT+CIPCLOSE?\r\n");
	LTE_Status result = processCommand(data, n);
	if(result==LTE_OK)
	{
		if(lte_response[1][11]=='0')
		{
			result=LTE_NO;
		}
	}
	return result;
}


static inline int lte_ping()
{
	char data[64];
	size_t n=sprintf(data,"AT\r\n");
	return processCommand(data, n);
}

static inline int lte_setBoudRate(int br)
{
	char data[64];
	size_t n=sprintf(data,"AT+IPR=%d \r\n",br);
	return processCommand(data, n);
}

static inline LTE_Status change_baudrate(uint32_t baudrate)
{
	LTE_Status result=LTE_NO;
	result=lte_setBoudRate(baudrate);
	if(result==LTE_NO)
	{
		uart_log("ERROR: LTE Module can't set boudrate %d \r\n", baudrate);
		return LTE_ERROR;
	}
	uart_log("LTE Module boudrate changed to %d, OK. \r\n", baudrate);

	HAL_UART_Abort(lte_param->huart);
	HAL_UART_DeInit(lte_param->huart);


	lte_param->huart->Init.BaudRate = baudrate;
	if(HAL_UART_Init(lte_param->huart)!=HAL_OK)
	{
		uart_log("UART_LTE can't set boudrate %d \r\n",baudrate);
		return LTE_ERROR;
	}
	uart_log("UART_LTE boudrate changed to %d, OK. \r\n", baudrate);
	return LTE_OK;
}

static inline LTE_Status lte_start()
{
	LTE_Status result=LTE_NO;
	if(lte_ping()==LTE_ERROR)
	{
		LOG("ERROR: Can't communicate with module \r\n");
		return LTE_ERROR;
	}
	uart_log("LTE Communication with 115000 boudrate oK, Changing boudrate... \r\n");

	if(change_baudrate(BAUD_RATE)!=LTE_OK)
		return LTE_ERROR;


	LOG("Checking transparent mode ... \r\n");
	if(lte_check_transparent()==LTE_NO)
	{
		if(lte_transparent()==LTE_ERROR)
		{
			uart_log("ERROR: Can't set transparent mode \r\n");
			return LTE_ERROR;
		}
		else
		{
			uart_log("Swiched in transparrent mode\r\n");
		}
	}

	transparent=1;

	LOG("Checking network ... \r\n");
	result=lte_check_net();
	LOG("Net State %d \r\n", result);
	switch(result)
	{
		case LTE_NO:
		{
			uart_log("Not open, opening.. \r\n");
			if(lte_netopen()!=LTE_OK)
			{
				uart_log("ERROR: Can't open \r\n");
				return LTE_ERROR;
			}
			//vTaskDelay(1000);
			if(lte_check_net()==LTE_OK)
			{
				uart_log("Opened. \r\n");
				if(lte_ipaddr()!=LTE_OK)
				{
					uart_log("Error: Can't check IP Address. \r\n");
					return LTE_ERROR;
				}
			}
			else
			{
				uart_log("ERROR: Can't open. \r\n");
				return LTE_ERROR;
			}
			return LTE_OK;
		}
		case LTE_ERROR:
		{
			uart_log("ERROR: Can't check network status \r\n");
			return LTE_ERROR;
		}
		case LTE_OK:
		{
			uart_log("Opened. \r\n");
			if(lte_ipaddr()!=LTE_OK)
			{
				uart_log("Error: Can't check IP Address. \r\n");
				return LTE_ERROR;
			}
			break;
		}
		default:
			return LTE_ERROR;

	}
	return result;

}

static inline int lte_connect()
{
	if(lte_tcp_open()!=1)
	{
		LOG("Error: Can't connect to server \r\n");
		return -1;
	}
	return 1;
}

static inline int lte_stop()
{
	/*
	if(recvcmplt==0)
	{
		if(HAL_UART_AbortReceive(lte_param->huart)!=HAL_OK)
		{
			uart_log("Error abort recive DMA \\r\n");
			return LTE_ERROR;
		}
	}
	*/
	if(transparent)
	{
		if(lte_endtransparent()==LTE_OK)
		{
			transparent=0;
			uart_log("End transparent mode\r\n");
		}
		else
		{
			uart_log("Error End transparent mode\r\n");
			return LTE_ERROR;
		}
	}

	if(lte_tcp_close()==LTE_OK)
	{
		uart_log("Disconnected from server\r\n");
		connected_to_server=0;
	}
	else
	{
		uart_log("Error disconnecting from server\r\n");
		return LTE_ERROR;
	}


	if(lte_netclose()==LTE_OK)
	{
		uart_log("Net closed\r\n");
	}
	else
	{
		uart_log("Error closing Net\r\n");
		return LTE_ERROR;
	}


	change_baudrate(115200);

	return LTE_OK;
}

static inline size_t lte_recv(char* data)
{
	return lte_getLine(data);
}

static inline void sendTerminalCommand()
{
	if(transparent==0)
	{
		if(strcmp(terminalbuf.buf,"ping")==0)
		{
			lte_ping();
		}
		else if(strcmp(terminalbuf.buf,"netopen")==0)
		{
			lte_netopen();
		}
		else if(strcmp(terminalbuf.buf,"netclose")==0)
		{
			lte_netclose();
		}
		else if(strcmp(terminalbuf.buf,"netcheck")==0)
		{
			if(lte_check_net()==LTE_OK)
			{
				uart_log("Connected to net \r\n");
			}
			else
			{
				uart_log("Not connected to net \r\n");
			}
		}
		else if(strcmp(terminalbuf.buf,"tcpopen")==0)
		{
			lte_tcp_open();
		}
		else if(strcmp(terminalbuf.buf,"tcpclose")==0)
		{
			lte_tcp_close();
		}
		else if(strcmp(terminalbuf.buf,"tcpcheck")==0)
		{
			if(lte_tcp_check()==LTE_OK)
			{
				uart_log("Connected to server \r\n");
			}
			else
			{
				uart_log("Not onnected to server \r\n");
			}
		}
		else if(strcmp(terminalbuf.buf,"ipaddr")==0)
		{
			lte_ipaddr();
		}
		else if(strcmp(terminalbuf.buf,"settrans")==0)
		{
			lte_transparent();
		}
		else if(strcmp(terminalbuf.buf,"endtrans")==0)
		{
			lte_endtransparent();
		}
		else if(strcmp(terminalbuf.buf,"brhi")==0)
		{
			change_baudrate(BAUD_RATE);
		}
		else if(strcmp(terminalbuf.buf,"brlo")==0)
		{
			change_baudrate(115200);
		}
		else
		{
			terminalbuf.buf[terminalbuf.size+1]='\r';
			terminalbuf.buf[terminalbuf.size+2]='\n';
			processCommand(terminalbuf.buf,terminalbuf.size+2);
			//uart_log("Invalid command \"%s\" %d\r\n",terminalbuf.buf,terminalbuf.size);
		}
	 }
	 else
	 {
		 send_audio(terminalbuf.buf,terminalbuf.size);
	 }
	 terminalbuf.size=0;
}

void lteTaskRun(void* param)
{
	lte_param=(LTE_PARAM*)param;


	//*
	if(lte_start()!=LTE_OK)
	{
		uart_log("Cannot open LTE network\r\n Please restartdevice. \r\n");
		return;
	}

	uart_log("LTE Task started\r\n");
	uart_log("Connecting to server ... \r\n");
	while(lte_tcp_open()!=LTE_OK)
	{
		vTaskDelay(2000);
	}
	connected_to_server=1;
	uart_log("OK, Connected to server. \r\n");
	//*/


	/*
	if(connected_to_server==1 && transparent==1)
	{
		lte_start_recv_DMA(lte_param->huart);
	}
	//*/


	for(;;)
	{
		//xSemaphoreTake(audioMutex,portMAX_DELAY);
		if(xSemaphoreTake(audioMutex,1)==pdTRUE)
		{
			if(connected_to_server==1 && ready_for_send==1)
			{
				HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin,1);
				send_audio((char*)readbuf,24);
				HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin,0);
				ready_for_send=0;
			}
			xSemaphoreGive(audioMutex);
		}

		/*
		if(connected_to_server==1 && transparent==1 && recvcmplt==1)
		{
			//serverHandler();
			HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin,0);
			lte_start_recv_DMA(lte_param->huart);
		}
		//*/


		 if(xSemaphoreTake(terminalMutex,1)==pdTRUE)
		 {
			 if(terminalbuf.size != 0)
			 {
				 sendTerminalCommand();
			 }
			 xSemaphoreGive(terminalMutex);
		 }

		 if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)==1)
		 {

			 uart_log("User Button clicked, transparent=%u stoping ...\r\n",transparent);
			 lte_stop();
			 vTaskDelay(100);
			 while(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)!=0);
			 uart_log("Stoped!\r\n");
		 }

		vTaskDelay(1);
	}
}
