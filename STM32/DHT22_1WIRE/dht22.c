/*
 * dht22.c
 *
 *  Created on: 28.02.2021
 *      Author: Krzysztof Półchłopek
 *
 */

#include "dht22.h"

void usDelay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&tim, 0);
	while ((__HAL_TIM_GET_COUNTER(&tim))<time);
}

void DHT_Init (TIM_HandleTypeDef *dht_tim)
{
	tim = *dht_tim;

	HAL_TIM_Base_Start(&tim);
}

void DHT_Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT_Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT_Start (void)
{
	DHT_Set_Pin_Output(DHT_PORT, DHT_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, GPIO_PIN_RESET);   // pull the pin low
	usDelay(1200);   // wait for > 1ms

	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, GPIO_PIN_SET);   // pull the pin high
	usDelay (20);   // wait for 20us

	DHT_Set_Pin_Input(DHT_PORT, DHT_PIN);   // set as input
}

uint8_t DHT_Check_Response (void)
{
	DHT_Set_Pin_Input(DHT_PORT, DHT_PIN);   // set as input
	uint8_t Response = 0;
	usDelay (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) // if the pin is low
	{
		usDelay (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) // if the pin is high, response is ok
		{
			Response = 1;
		}
		else
		{
			Response = -1;
		}
	}

	while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go low
	return Response;
}

uint8_t DHT_Recv_Byte (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go high
		usDelay (40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));  // wait for the pin to go low
	}

	return i;
}

bool DHT_Read (float *Temperature, float *Humidity)
{
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
	uint16_t RH, TEMP;

	DHT_Start();
	if (DHT_Check_Response() > 0)
	{
		Rh_byte1 = DHT_Recv_Byte ();
		Rh_byte2 = DHT_Recv_Byte ();
		Temp_byte1 = DHT_Recv_Byte ();
		Temp_byte2 = DHT_Recv_Byte ();
		SUM = DHT_Recv_Byte();

		if ((uint8_t)(Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2) == SUM)
		{
			TEMP = ((Temp_byte1<<8)|Temp_byte2);
			RH = ((Rh_byte1<<8)|Rh_byte2);

			*Temperature = (float) (TEMP/10.0);
			*Humidity = (float) (RH/10.0);

			return true;
		}
	}
	return false;
}
