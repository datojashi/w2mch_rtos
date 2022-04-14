/**
  ******************************************************************************
  * @file           : uart_logger.h
  * @brief          : Header for uart_logger.c file.
  *                   This file contains function declarations for logging throw UART.
  ******************************************************************************
 */

#include "uart_logger.h"

static UART_HandleTypeDef* phuart;

char* prompt;

void uart_logger_init(UART_HandleTypeDef* _phuart, char* p)
{
	phuart=_phuart;
	loging=1;
	prompt=p;
}

void uart_log(char* format, ... )
{
	va_list ap;
	uint8_t buf[128];
	va_start(ap, format);
	int n = vsnprintf ((char*)buf, 128, format, ap);
	va_end(ap);
	HAL_UART_Transmit(phuart, (unsigned char*)buf, n, 100);
}

void uart_log_prompt()
{
	uart_log("%s",prompt);
}


HAL_StatusTypeDef uart_getchar(uint8_t* ch,  uint8_t* sz)
{
	HAL_StatusTypeDef result=HAL_ERROR;
	result = HAL_UART_Receive(phuart, ch, 1, 10000);
	if(result == HAL_OK)
	{
		if( (ch[0] >= 32 && ch[0] < 126) || ch[0]==127 || ch[0]=='\n' || ch[0]=='\r' || ch[0]=='\b')
		{
			*sz=1;
			return result;
		}
		if(*ch==27 || *ch==126)
		{
			result=HAL_UART_Receive(phuart, ch+1, 2, 10);
			if(result == HAL_OK)
			{
				*sz=3;
			}
		}
		else
			result=HAL_ERROR;
	}
	return result;

}

void uart_cursorLeft(int n)
{
	for(int i=0; i < n; i++)
	{
		uart_log("%c%c%c",27,91,68);
	}
}

void uart_cursorRight(int n)
{
	for(int i=0; i < n; i++)
	{
		uart_log("%c%c%c",27,91,67);
	}
}

void uart_clearString(int n)
{
	for(int i=0; i<n; i++)
	{
		printf(" ");
	}
}

uint8_t uart_recv(char* data, size_t len)
{
        uint8_t c[3], t=0;
        uint8_t sz;
        HAL_StatusTypeDef hstat;
        memset(data,0,len);
        while((c[0]!='\n' && c[0]!='\r') && t<len)
        {
                hstat = uart_getchar(c,  &sz);//HAL_UART_Receive(phuart, &c, 1, 10);
                if(hstat == HAL_OK)
                {
                	if(sz==1)
                	{
                		switch(c[0])
                		{
                		case 10:
                		case 13:
                		{
                			break;
                		}
                		case 32:
                		{
                			int a=strlen(data);
                			if(t>=a)
                			{
                				data[t++]=32;
                				break;
                			}
                			for(int i=a; i>=t; i--)
                			{
                				data[i+1]=data[i];
                			}
                			data[t]=32;
                			break;
                		}
                		case 8://bs
                		{
                			c[1]=32;
                			c[2]=8;
                			sz=3;
                			t--;
                			data[t]=32;
                			break;
                		}
                		case 127://bs
                		{
                			if(t > 0)
                			{
                				data[--t]=32;
                			}
                			else
                				c[0]=0;
                			break;
                		}
                		default:
                			data[t++]=c[0];
                		}
                		uart_log("\r%s%s",prompt,data);
                		uart_cursorLeft(strlen(data)-t);
                	}
                	else
                	{
                		switch(c[2])
                		{
                		case 51:
                		{
                			int i;
                			if(c[0]==27)
                			{
                				for(i=t; i<strlen(data); i++)
                				{
                					data[i]=data[i+1];
                				}
                				data[i]=0;
                				//uart_cursorLeft(t);
                				//uart_clearString(strlen(data)+1);

                				uart_log("\r%s            ",prompt);
                				uart_log("\r%s%s",prompt,data);
                				//uart_log("\rw2mch                   ");
                				//uart_log("\rw2mch>%s",data);

                				uart_cursorLeft(strlen(data)-t);
                			}
                			break;
                		}
                		case 67: //right
                		{
                			if(t<strlen(data))
                			{
                				uart_cursorRight(1);
                				t++;
                			}
                			break;
						}
                		case 68: //left
                		{
                			if(t > 0)
                			{
                				uart_cursorLeft(1);
                				t--;
                			}
                			break;
                		}
                		}
                	}
                }
        }
        uart_log("\r\n");
        return t;
}

void printHex(char* data, int size)
{
	  for(int i=0; i<size; i++)
	   {
	       uart_log("%02x", data[i]);
	   }
	   uart_log("\r\n");
}

