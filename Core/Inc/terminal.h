/*
 * terminal.h
 *
 *  Created on: Dec 16, 2021
 *      Author: dato
 */

#ifndef INC_TERMINAL_H_
#define INC_TERMINAL_H_

typedef struct
{
	UART_HandleTypeDef* huart;
}TERMINAL_PARAM;

void terminalRun();

#endif /* INC_TERMINAL_H_ */
