/*
 * dht22.h
 *
 *  Created on: 28.02.2021
 *      Author: Krzysztof Półchłopek
 *
 */

#ifndef _DHT22_H_
#define _DHT22_H_

#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "tim.h"

#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_1

TIM_HandleTypeDef tim;

void usDelay (uint16_t time);
void DHT_Init (TIM_HandleTypeDef *dht_tim);
void DHT_Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT_Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT_Start (void);
uint8_t DHT_Check_Response (void);
uint8_t DHT_Recv (void);
bool DHT_Read (float *Temperature, float *Humidity);

#endif
