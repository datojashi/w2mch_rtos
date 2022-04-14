/*
 * audio.h
 *
 *  Created on: Feb 4, 2022
 *      Author: dato
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#define ALAW_BUFFER_SECTORS 24U
#define ALAW_BUFFER_SIZE (ALAW_BUFFER_SECTORS*512)
#define SAMPLE_BUFFER_SIZE (ALAW_BUFFER_SIZE*4)


#define	PCM16	1
#define	ALAW	6
#define	ULAW	7

#define READ_SECTORS_NUMBER	ALAW_BUFFER_SECTORS

typedef struct
{
	ADC_HandleTypeDef* hadc;
	SD_HandleTypeDef* hsd;
	uint8_t * const sram1;
	uint8_t * const sram2;
}AUDIO_PARAM;

typedef struct
{
	uint16_t sampleRate;
	uint8_t format;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	uint8_t bytesPerSample;
	uint32_t totalSectors; //total blocks
}REC_SETTS;


void audioTaskRun();

#endif /* INC_AUDIO_H_ */
