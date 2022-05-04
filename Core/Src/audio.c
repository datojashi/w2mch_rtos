/*
 * audio.c
 *
 *  Created on: Feb 4, 2022
 *      Author: dato
 */

#include "globals.h"
#include "audio.h"
#include "alawtable.h"

#include "sdio.h"

#define SETTINGS_SECTOR	128*512
//#define META_SECTOR	256*512
//#define CLOCK_SECTOR	200000
//#define AVG_SAMPLE_SECTOR 500000
#define DATA_SECTOR	1000000

AUDIO_PARAM* audio_param;
REC_SETTS rec_setts;

volatile static int current_sector=DATA_SECTOR;
volatile static int current_read_sector=DATA_SECTOR;

uint16_t* samplebuf;
uint8_t* samplebufByte;

uint8_t* readbuf;
uint8_t* alawbuf;

uint8_t channels_number=1;
volatile int samplebuf_offset=0;
volatile uint8_t sampleBufReady=0;
volatile uint8_t wrComplete=0;

volatile uint8_t send_flag=0;

volatile uint16_t signal_level[4]={2048,2048,2048,2048};


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

	sampleBufReady=1;
	samplebuf_offset=0;
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	sampleBufReady=1;
	samplebuf_offset=ALAW_BUFFER_SIZE;
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

static inline  void getSignalLevel()
{
	uint32_t sum[4]={0,0,0,0};
	uint16_t ct[4]={0,0,0,0};
	for(int i=0; i<ALAW_BUFFER_SIZE; i++)
	{

		uint8_t ch_index=i%channels_number;
		sum[ch_index]=sum[ch_index]+(samplebuf[samplebuf_offset+i]&0x0fff);
		ct[ch_index]++;
	}
	for(int i=0; i<channels_number; i++)
	{
		signal_level[i]=sum[i]/ct[i];
	}
}

static inline  void aLawEncode()
{
	uint8_t cypher_ct=0;
	for(int i=0; i<ALAW_BUFFER_SIZE; i++)
	{
		uint16_t sample=samplebuf[samplebuf_offset+i]&0x0fff;
		int16_t d=sample-signal_level[i%channels_number];
		uint16_t dabs=abs(d);

		if(d >= 0)
			alawbuf[i]=alaw_p[dabs]^0x55;//^cypher_a[cypher_ct++];
		else
			alawbuf[i]=alaw_n[dabs]^0x55;//^cypher_a[cypher_ct++];

		if(cypher_ct==16)
			cypher_ct=0;
	}
}

static uint8_t  writeAudioData()
{
	if( (current_sector+ALAW_BUFFER_SECTORS) < rec_setts.totalSectors)
	{
		wrComplete=0;
		HAL_StatusTypeDef stat=writeBlocks(audio_param->hsd, alawbuf, current_sector, ALAW_BUFFER_SECTORS);
		if(stat==HAL_OK)
		{
			//LOG("Writen sector %d, OK \r\n", current_sector);
			current_sector=current_sector+ALAW_BUFFER_SECTORS;
			//HAL_Delay(1);
			//writeMetaData();
			return 1;
		}
		else
		{
			//current_sector=current_sector+ALAW_BUFFER_SECTORS;
			LOG("Disk write error! current_sector=%d, sct=%d, status=%d, error=%d \r\n",current_sector,ALAW_BUFFER_SECTORS,stat,audio_param->hsd->ErrorCode);
			return 0;
		}
	}
	else
	{
		LOG("Disk overflow! current_sector=%d, sct=%d, totalsectors=%d \r\n", current_sector, ALAW_BUFFER_SIZE/512, rec_setts.totalSectors);
		return 0;
	}
}


static inline uint8_t  readAudioData()
{
	int result=0;
	if(current_sector > (current_read_sector + READ_SECTORS_NUMBER))
	{
		HAL_StatusTypeDef stat=readBlocks(audio_param->hsd, readbuf, current_read_sector, READ_SECTORS_NUMBER);
		if(stat==HAL_OK)
		{
			current_read_sector=current_read_sector+READ_SECTORS_NUMBER;
			result=1;
		}
		else
		{
			LOG("Disk read error! current_read_sector=%d, status=%d, error=%d\r\n",current_read_sector, stat, audio_param->hsd->ErrorCode);
			result = 2;
		}
	}
	return result;
}

static inline int sendAudioData()
{
	int result = 0;

	return result;
}


static inline void configureRecorder()
{
	rec_setts.totalSectors=100000000;
	current_sector=DATA_SECTOR;
}

void audioTaskRun(void* param)
{
	audio_param=(AUDIO_PARAM*)param;
	configureRecorder();
	alawbuf = (uint8_t*)malloc(ALAW_BUFFER_SIZE);

	samplebuf=(uint16_t*)(audio_param->sram1+0x30000);
	samplebufByte=(audio_param->sram1+0x30000);

	readbuf=(audio_param->sram1+0x40000);




	//sendcmd[0]=0xaa;
	//sendcmd[1]=0x55;

	//LOG("SRAM1 = %x",audio_param->sram1);

	channels_number=audio_param->hadc->Init.NbrOfConversion;

	HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, 0);
	vTaskDelay(2000);

	HAL_StatusTypeDef result=HAL_ADC_Start_DMA(audio_param->hadc, (uint32_t*)samplebuf, SAMPLE_BUFFER_SIZE/2);
	//HAL_StatusTypeDef result=HAL_ADC_Start_IT(audio_param->hadc);
	if(result==HAL_OK)
	{
		  uart_log("ADC started in DMA mode, %d Channels, OK \r\n",channels_number);
	  	  uart_log("Measuring zero level, please keep silence ... \r\n");
	  	  while(!sampleBufReady);
	  	  getSignalLevel();
	  	  sampleBufReady=0;
	  	  for(int i=0; i<channels_number; i++)
	  		  uart_log("signal_level[%d]=%d \r\n",i,signal_level[i]);
	}
	else
	{
	  	  uart_log("ADC starting error %d OK \r\n",result);
	  	  Error_Handler();
	}


	LOG("Audio Task started, %d Channels\r\n", channels_number);

	while(1)
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
		if(sampleBufReady)
		{

			HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);
			aLawEncode();
#ifdef __SD__
			if(writeAudioData()==0)
			{
				//HAL_SD_Init(audio_param->hsd);
				//audio_param->hsd->ErrorCode=0;
				LOG("Disk write error, abort!\r\n");
				HAL_SD_Abort(audio_param->hsd);
				//HAL_SD_Init(audio_param->hsd);
			}
#endif
			sampleBufReady=0;
		}
		if(xSemaphoreTake(audioMutex,1)==pdTRUE)
		{
#ifdef __SD__
			if(send_flag==0)
				send_flag=readAudioData();
			xSemaphoreGive(audioMutex);
			if(send_flag==2)
			{
				LOG("Disk read error, Abort!\r\n");
				HAL_SD_Abort(audio_param->hsd);
				send_flag=0;
				//HAL_SD_Init(audio_param->hsd);
			}
#else
			if(send_flag==0)
			{
				memcpy(readbuf,alawbuf,ALAW_BUFFER_SIZE);
				memset(readbuf, 0xff, ALAW_BUFFER_SIZE);
				send_flag=1;
			}
			xSemaphoreGive(audioMutex);
			while(sampleBufReady==0) vTaskDelay(1);
#endif
		}
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);
		vTaskDelay(1);
	}
}
