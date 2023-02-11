/*
 * leddriver.c
 *
 *  Created on: Apr 1, 2021
 *      Author: verdan
 */
#include "stm32f4xx_hal.h"

void led_on(void)
{
	  GPIOD->ODR |= 1 << 12;
	  GPIOD->ODR |= 1 << 13;
	  GPIOD->ODR |= 1 << 14;
	  GPIOD->ODR |= 1 << 15;
}
void led_of(void)
{
	  GPIOD->ODR &= ~(1 << 12);
	  GPIOD->ODR &= ~(1 << 13);
	  GPIOD->ODR &= ~(1 << 14);
	  GPIOD->ODR &= ~(1 << 15);
}

