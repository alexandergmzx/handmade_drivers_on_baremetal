/*
 * 001led_toggle.c
 *
 *  Created on: 22 mar. 2020
 *      Author: Alexander Gomez
 */
#include "./inc/stm32f413xx.h"

int main(void) {
	GPIO_Handle_t led0;
	GPIO_Handle_t led7;
	GPIO_Handle_t led14;
	GPIO_clk(GPIOB, ENABLE);
	led0.pGPIOx = GPIOB;
	led0.GPIO_PinConfig.GPIO_PinNumber = 0;
	led0.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led0.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	led0.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	led0.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_init(&led0);

	led7.pGPIOx = GPIOB;
	led7.GPIO_PinConfig.GPIO_PinNumber = 7;
	led7.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led7.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_init(&led7);


	led14.pGPIOx = GPIOB;
	led14.GPIO_PinConfig.GPIO_PinNumber = 14;
	led14.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led14.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_init(&led14);


	while(TRUE){
		GPIO_togglepin(GPIOB, 0);
		delay(10);
		GPIO_togglepin(GPIOB, 7);
		delay(20);
		GPIO_togglepin(GPIOB, 14);
		delay(30);
	}

	return 0;
}

