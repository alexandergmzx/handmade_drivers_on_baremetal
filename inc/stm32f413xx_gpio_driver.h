/*
 * stm32f413xx_gpio_driver.h
 *
 *  Created on: 20 mar. 2020
 *      Author: Alexander Gomez
 */

#ifndef INC_STM32F413XX_GPIO_DRIVER_H_
#define INC_STM32F413XX_GPIO_DRIVER_H_

#include "stm32f413xx.h"

//Structure to hold the Configuration of each GPIO pin
typedef struct
{
	uint8_t GPIO_PinNumber;		/*< Possible values from 		@GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;  		/*< Possible modes from 		@GPIO_PIN_MODES  >*/
	uint8_t GPIO_PinSpeed;		/*< Possible speeds from 		@GPIO_PIN_SPEED	 >*/
	uint8_t GPIO_PinPuPdControl;/*< Possible values from  		@GPIO_PIN_PUPD	 >*/
	uint8_t GPIO_PinOPType;		/*< Possible output types from 	@GPIO_PIN_OTYPES >*/
	uint8_t GPIO_PinAltFunMode; /*< Every pin has an specific set of AF modes	 >*/
}GPIO_PinConfig_t;

//Structure to handle the base adddress of the peripheral and its config of each pin
typedef struct
{//pointer to hold the base address of the GPIO Peripheral
	GPIO_RegDef_t *pGPIOx; //GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;//Configuration of the pin
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
/*
enum gpio_numbers{
	GPIO_PIN_NO_0,
	GPIO_PIN_NO_1,
	GPIO_PIN_NO_2,
	GPIO_PIN_NO_3,
	GPIO_PIN_NO_4,
	GPIO_PIN_NO_5,
	GPIO_PIN_NO_6,
	GPIO_PIN_NO_7,
	GPIO_PIN_NO_8,
	GPIO_PIN_NO_9,
	GPIO_PIN_NO_10,
	GPIO_PIN_NO_11,
	GPIO_PIN_NO_12,
	GPIO_PIN_NO_13,
	GPIO_PIN_NO_14,
	GPIO_PIN_NO_15,
};
*/
/*
 * @GPIO_PIN_MODES
 * GPIO possible modes
*/
enum gpio_modes{
	GPIO_MODE_IN,
	GPIO_MODE_OUT,
	GPIO_MODE_AF,//alternte function mode
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,//Interrupt Falling Edge Trigger
	GPIO_MODE_IT_RT,//Interrupt Rising Edge Trigger
	GPIO_MODE_IT_RFT,//Interrupt Rising-Falling Edge Trigger
};

/*
 * @GPIO_PIN_OTYPES
 * GPIO pin possible output types
 */
enum gpio_otypes{
	GPIO_OP_TYPE_PP,
	GPIO_OP_TYPE_OD,
};

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
enum gpio_speed{
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_FAST,
	GPIO_SPEED_HIGH,
};

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
enum gpio_pupd{
	GPIO_NO_PUPD,
	GPIO_PIN_PU,
	GPIO_PIN_PD,
};

/*******************************************************************************
 * 				APIs supported by this driver
 * For more information about the functionality check the function definitions
 * ***************************************************************************** */

//periphertal clock control
void GPIO_clk(GPIO_RegDef_t *pGPIOx, uint8_t enable_disable);

//Function to de/initialize the given GPIO port and Pin
void GPIO_init(GPIO_Handle_t *pGPIOHandle);//Needs all the function Handle
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);//Just needs the reset register

//Functions to read/write to pins
uint8_t GPIO_readpin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);//returns a boolean
uint16_t GPIO_readport(GPIO_RegDef_t *pGPIOx);//returns the full content of the IDR
void GPIO_writepin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t value);
void GPIO_writeport(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_togglepin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);
void GPIO_toggleport(GPIO_RegDef_t *pGPIOx);

//functions for interrupt handling
void GPIO_IRQconfig(uint8_t IRQnumber,uint8_t enable_disable);
void GPIO_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQhandling(uint8_t pin_number);

/*
 * The NUCLEO-144 board initializers
 */
void GPIO_BoardLEDSInit(void);
void GPIO_BoardButtonInit(void);

/*
 * SmartGPU2/arduino special functions
 */
enum ButtonStates { UP, DOWN, PRESS, RELEASE };

uint8_t delay_debounce(enum ButtonStates button_state);

enum pin_arduino{
	p13=5,//PA5 is equal to the pin D13 on the arduino
};

void delay(uint32_t value);

void digitalWrite(uint8_t pin_number, uint8_t value);

#endif /* INC_STM32F413XX_GPIO_DRIVER_H_ */
