/*
 * Communication.h
 *
 *  Created on: Sep 28, 2022
 *      Author: NAWAT.K
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef STM32F7XX_H
#include "stm32f7xx_hal.h"
#endif

#ifndef STM32F7XX_HAL_UART_H
#include "stm32f7xx_hal_uart.h"
#endif

#define Header_OBC		0xAA
#define Header_EPS		0x55

enum CHBIT{
	CONFIG_CH0	=		0x01,
	CONFIG_CH1	=		0x02,
	CONFIG_CH2	=		0x04,
	CONFIG_CH3	=		0x08,
	CONFIG_CH4	=		0x10,
	CONFIG_CH5	=		0x20,
	CONFIG_CH6	=		0x40,
	CONFIG_CH7	=		0x80,
};


enum RegAddr{
	CONFIG		=		0X00,
	CH_Voltage	=		0x01,
	CH_Current	=		0x02,
	BATT_CELL	=		0x03,
	PACK_Voltage=		0x04,
	PACK_Current=		0x05
};

enum CHREAD{
	CH0			=		0x00,
	CH1			=		0x01,
	CH2			=		0x02,
	CH3			=		0x03,
	CH4			=		0x04,
	CH5			=		0X05,
	CH6			=		0X06,
	CH7			=		0X07,
	CH8			=		0X08   // 3.3V automaticly on upon device startup Always-ON
};

enum PACK{
	Read_Cell	=		0x00,
	Read_Pack	=		0x01
};

enum Voltage_Current{
	Read_Voltage	=	0x00,
	Read_Current	=	0x01
};

enum cell_COMM{
	Cell0		=		0x00,
	Cell1		=		0x01,
	Cell2		=		0x02,
	Cell3		=		0x03
};


typedef struct {
	UART_HandleTypeDef	*uartHandle;
	GPIO_TypeDef		*OBCPort;
	uint16_t			OBCPin;

	GPIO_TypeDef		*CH0_Port;
	uint16_t			CH0_Pin;

	GPIO_TypeDef		*CH1_Port;
	uint16_t			CH1_Pin;

	GPIO_TypeDef		*CH2_Port;
	uint16_t			CH2_Pin;

	GPIO_TypeDef		*CH3_Port;
	uint16_t			CH3_Pin;

	GPIO_TypeDef		*CH4_Port;
	uint16_t			CH4_Pin;

	GPIO_TypeDef		*CH5_Port;
	uint16_t			CH5_Pin;

	GPIO_TypeDef		*CH6_Port;
	uint16_t			CH6_Pin;

	GPIO_TypeDef		*CH7_Port;
	uint16_t			CH7_Pin;

	GPIO_TypeDef		*CH8_Port;
	uint16_t			CH8_Pin;
	/* Add other here*/
	uint8_t RxBuf[4];
	uint16_t RawVCH[9];
	uint16_t RawICH[9];
	uint16_t RawBattCell[4];
	uint16_t RawBattpack;
	short RawCurrent;
}Commu_t;

void EPS_ENABLE_Channal(Commu_t *COM,uint8_t CH);
void EPS_DISABLE_Channal(Commu_t *COM,uint8_t CH);

uint8_t EPS_ReadMessageDMA(Commu_t *COM);
void 	EPS_ReadMessageDMA_Complete(Commu_t *COM);

#endif /* INC_COMMUNICATION_H_ */
