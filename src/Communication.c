/*
 * Communication.c
 *
 *  Created on: Sep 28, 2022
 *      Author: NAWAT.K
 */
#include "Communication.h"
#include "BQ76920.h"
#include "MAX11128.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>


void EPS_ENABLE_Channal(Commu_t *COM,uint8_t CH)
{
	switch(CH)
	{
	case CH0:
		HAL_GPIO_WritePin(COM->CH0_Port, COM->CH0_Pin, GPIO_PIN_SET);
		break;
	case CH1:
		HAL_GPIO_WritePin(COM->CH1_Port, COM->CH1_Pin, GPIO_PIN_SET);
		break;
	case CH2:
		HAL_GPIO_WritePin(COM->CH2_Port, COM->CH2_Pin, GPIO_PIN_SET);
		break;
	case CH3:
		HAL_GPIO_WritePin(COM->CH3_Port, COM->CH3_Pin, GPIO_PIN_SET);
		break;
	case CH4:
		HAL_GPIO_WritePin(COM->CH4_Port, COM->CH4_Pin, GPIO_PIN_SET);
		break;
	case CH5:
		HAL_GPIO_WritePin(COM->CH5_Port, COM->CH5_Pin, GPIO_PIN_SET);
		break;
	case CH6:
		HAL_GPIO_WritePin(COM->CH6_Port, COM->CH6_Pin, GPIO_PIN_SET);
		break;
	case CH7:
		HAL_GPIO_WritePin(COM->CH7_Port, COM->CH7_Pin, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}


void EPS_DISABLE_Channal(Commu_t *COM,uint8_t CH)
{
	switch(CH)
	{
	case CH0:
		HAL_GPIO_WritePin(COM->CH0_Port, COM->CH0_Pin, GPIO_PIN_RESET);
		break;
	case CH1:
		HAL_GPIO_WritePin(COM->CH1_Port, COM->CH1_Pin, GPIO_PIN_RESET);
		break;
	case CH2:
		HAL_GPIO_WritePin(COM->CH2_Port, COM->CH2_Pin, GPIO_PIN_RESET);
		break;
	case CH3:
		HAL_GPIO_WritePin(COM->CH3_Port, COM->CH3_Pin, GPIO_PIN_RESET);
		break;
	case CH4:
		HAL_GPIO_WritePin(COM->CH4_Port, COM->CH4_Pin, GPIO_PIN_RESET);
		break;
	case CH5:
		HAL_GPIO_WritePin(COM->CH5_Port, COM->CH5_Pin, GPIO_PIN_RESET);
		break;
	case CH6:
		HAL_GPIO_WritePin(COM->CH6_Port, COM->CH6_Pin, GPIO_PIN_RESET);
		break;
	case CH7:
		HAL_GPIO_WritePin(COM->CH7_Port, COM->CH7_Pin, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}


uint8_t EPS_ReadMessageDMA(Commu_t *COM)
{
	if (HAL_UART_Receive_DMA(COM -> uartHandle, COM->RxBuf, sizeof(COM -> RxBuf))== HAL_OK)
	{
		return 1;
	}
	else
		return 0;
}



void EPS_ReadMessageDMA_Complete(Commu_t *COM)
{
	uint8_t TxBuf[4];
	/* process received message */
	if((COM->RxBuf[0]) == Header_OBC)
	{

		TxBuf[0] = Header_EPS;
		uint8_t CH;
		uint8_t cell;
		switch(COM->RxBuf[1])	// Register Address
		{
		case CONFIG:
			for(int i=0;i<=7;i++)
			{
				if(((COM->RxBuf[3])&(1<<i))==(1<<i))
				{
					EPS_ENABLE_Channal(COM,i);
					// set TX buffer
				}
				else
					EPS_DISABLE_Channal(COM,i);
			}
			TxBuf[1] = CONFIG;
			TxBuf[2] = COM->RxBuf[2];  // STATUS and BATT report
			TxBuf[3] = COM->RxBuf[3];
			break;
		case CH_Voltage:
			CH = ((COM->RxBuf[2])&0xF0) >> 4;
			TxBuf[1] = CH_Voltage;
			TxBuf[2] = (uint8_t)((COM->RawVCH[CH]&0x0F00) >> 8);
			TxBuf[3] = (uint8_t) (COM->RawVCH[CH]&0x00FF);
			break;
		case CH_Current:
			CH = ((COM->RxBuf[2])&0xF0) >> 4;
			TxBuf[1] = CH_Current;
			TxBuf[2] = (uint8_t)((COM->RawICH[CH]&0x0F00) >> 8);
			TxBuf[3] = (uint8_t) (COM->RawICH[CH]&0x00FF);
			break;
		case BATT_CELL:
			cell = ((COM->RxBuf[2])&0xC0) >> 6;
			TxBuf[1] = BATT_CELL;
			TxBuf[2] = (uint8_t)((COM->RawBattCell[cell]&0x0F00) >> 8);
			TxBuf[3] = (uint8_t) (COM->RawBattCell[cell]&0x00FF);
			break;
		case PACK_Voltage:
			TxBuf[1] = PACK_Voltage;
			TxBuf[2] = (uint8_t)((COM->RawBattpack&0x0F00) >> 8);
			TxBuf[3] = (uint8_t) (COM->RawBattpack&0x00FF);
			break;
		case PACK_Current:
			TxBuf[1] = PACK_Current;
			TxBuf[2] = (uint8_t)((COM->RawCurrent&0x0F00) >> 8);
			TxBuf[3] = (uint8_t) (COM->RawCurrent&0x00FF);
			break;
		default:
			break;
		}
		HAL_UART_Transmit(COM -> uartHandle, TxBuf, sizeof(TxBuf), 100);
		// UART Transmit data
	}
	else
		// UART Transmit 0
		return;
}
