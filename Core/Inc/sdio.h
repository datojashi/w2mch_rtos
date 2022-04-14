#ifndef __SDIO_H
#define __SDIO_H

#include "stm32f7xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"
//#include "main.h"

#define MAX_POLLING	100

inline static uint32_t polling(SD_HandleTypeDef *hsd)
{
	int st=0;
	for(uint32_t i=0; i<MAX_POLLING; i++)
	{

		    st=HAL_SD_GetCardState(hsd);
		   	if(st==HAL_SD_CARD_TRANSFER)
		   	{
		   		return 1;
		   	}
		   	else
		   	{
		   		HAL_Delay(1);
		   	}
	 }
	return 0;
}

inline static HAL_StatusTypeDef readBlocks(SD_HandleTypeDef *hsd, uint8_t *buf, uint32_t sector, uint32_t secNmb)
{
	if(!polling(hsd))
		return HAL_ERROR;
	else
	{
		return HAL_SD_ReadBlocks(hsd, buf, sector, secNmb, 1000);
	}
}


inline static HAL_StatusTypeDef writeBlocks(SD_HandleTypeDef *hsd, uint8_t *buf, uint32_t sector, uint32_t secNmb)
{
	HAL_StatusTypeDef res=HAL_OK;
	if(polling(hsd))
	{
		res=HAL_SD_WriteBlocks(hsd, buf, sector, secNmb,1000);
	}
	else
	{
		res=7;
	}
	return res;
}

inline static HAL_StatusTypeDef writeBlocksDMA(SD_HandleTypeDef *hsd, uint8_t *buf, uint32_t sector, uint32_t secNmb)
{
	HAL_StatusTypeDef res=HAL_OK;
	if(polling(hsd))
	{
		res=HAL_SD_WriteBlocks_DMA(hsd, buf, sector, secNmb);
	}
	else
	{
		res=7;
	}
	return res;
}

#endif
