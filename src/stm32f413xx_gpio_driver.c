/*
 * File: stm32f413xx_gpio_driver.c
 *
 *  Created on: 20 mar. 2020
 *      Author: Alexander Gomez
 *
 *      @brief - Part of the course
 *      "Mastering Microcontroller with Embedded Driver Development"
 */

#include "../inc/stm32f413xx_gpio_driver.h"

/****************************************************************************
 * @fn - GPIO_PeriClockControl
 * @brief - This function enables or disables peripheral
 * 			clock for the given GPIO port
 * @param[*pGPIOx] - Pointer to the Register Definition of the Specified GPIO
 * @param[enable_disable] - Macro to ENABLE or DISABLE the Clock (EnorDi)
 ****************************************************************************/
void GPIO_clk(GPIO_RegDef_t *pGPIOx, uint8_t enable_disable)
{
	if (enable_disable == ENABLE) {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else if(enable_disable == DISABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn		- GPIO_init
 *
 * @brief	- This function initializes the given GPIO pin structure
 *
 * @param[pGPIOHandle] - Pointer to GPIO Handle structure
 *****************************************************************/
void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{
	//START THE CLOCK, DONT LEAVE IT UN-INITIALIZED
	GPIO_clk(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0; //Temporal register for allocations
	//1.Configure the mode of the pin	(MODER)
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{ //get normal modes "¿Qué dijiste de mi MODER?"
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else{//it is an interrupt mode

		//0.Set pin in Input mode configuration
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3
						<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//1.Configure the Trigger Selection Register
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{//Configure the FTSR (Falling Trigger Selection Register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{//Configure the RTSR (Rising Trigger Selection Register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{//Configure both RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.Configure the GPIO port selection in SYSCFG_EXTI Control Register
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 4));


		//3.Enable the EXTI interrupt delivery using IMR(Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2.Configure the output speed				(OSPEEDR)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3
			<< ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3.Configure the pU-pD settings	(PUPDR)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3
			<< ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4.Configure the Output Type		(OTYPER)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5.Configure the Alt Functionality	(AFR[2])
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{//configure the Alternate function modes
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
}
/*****************************************************************
 * @fn		- GPIO_deinit
 *
 * @brief	- This function de-initialize the given GPIO peripheral
 *
 * @param[pGPIOx] - Base address of the GPIO peripheral
 *****************************************************************/
void GPIO_deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Pin/port read and write
 */
/*****************************************************************
 * @fn		- GPIO_readpin
 *
 * @brief	- This function reads value of input pin, on
 * 			  a specific port
 *
 * @param[pGPIOx]		- Base address of the GPIO peripheral
 * @param[pin_number]	- Pin number
 *
 * @return	- Digital value of the pin
 *****************************************************************/
uint8_t GPIO_readpin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
	uint8_t value = 0;
	value = (uint8_t)(pGPIOx->IDR >> pin_number) & 0x1;
	return value;
}
/*****************************************************************
 * @fn		- GPIO_readport
 *
 * @brief	- This function reads value of input port
 *
 * @param[pGPIOx] - Base address of the GPIO peripheral
 *
 * @return	- Value read from the selected port
 *****************************************************************/
uint16_t GPIO_readport(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
/*****************************************************************
 * @fn		- GPIO_writepin
 *
 * @brief	- This function writes a digital value on a specific
 * 			  output pin
 *
 * @param[pGPIOx]		- Base address of the GPIO peripheral
 * @param[pin_number]	- Pin number
 * @param[value]		- Digital Value (Set/Reset)
 *****************************************************************/
void GPIO_writepin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t value)
{
	if (value == SET)
	{//write 1 to the Output Data Register at the bit field of the pin
		pGPIOx->ODR |= (1 << pin_number);
	} else if (value == RESET){
		pGPIOx->ODR &= ~(1 << pin_number);
	}
}
/*****************************************************************
 * @fn			- GPIO_writeport
 *
 * @brief		- This function writes a value on a specific
 * 			  output port
 *
 * @param[pGPIOx]	- Base address of the GPIO peripheral
 * @param[value]	- 16 bit value
 *****************************************************************/
void GPIO_writeport(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}
/*****************************************************************
 * @fn		- GPIO_togglepin
 *
 * @brief	- This function toggles the current value in a
 * 				specific output pin
 *
 * @param[pGPIOx]		- Base address of the GPIO peripheral
 * @param[pin_number]	- Pin number
 *****************************************************************/
void GPIO_togglepin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
	pGPIOx->ODR  ^= ( 1 << pin_number);
}
/*****************************************************************
 * @fn		- GPIO_toggleport
 *
 * @brief	- This function toggles the current value in a
 * 				specific output port
 *
 * @param[pGPIOx] - Base address of the GPIO peripheral
 *****************************************************************/
void GPIO_toggleport(GPIO_RegDef_t *pGPIOx){
	pGPIOx->ODR ^= 0xFFFFFFFF;
}

/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn		- GPIO_IRQConfig
 *
 * @brief	- This function Sets/clears an interrupt in the NVIC
 *
 * @param[IRQnumber] 		- IRQ Interrupt number
 * @param[enable_disable]	- Macro: Enable/Disable
 *****************************************************************/
void GPIO_IRQconfig(uint8_t IRQnumber, uint8_t enable_disable)
{
	if (enable_disable == ENABLE)
	{//Configure ISERx (Interrupt Set-Enable Registers)
		if (IRQnumber < 32) {//ISER0 register
			*NVIC_ISER0 |= (1 << IRQnumber);
		}else if (IRQnumber >= 32 && IRQnumber < 64) {//ISER1 register
			*NVIC_ISER1 |= (1 << IRQnumber%32);
		}else if (IRQnumber >= 64 && IRQnumber < 96) {//ISER2 register
			*NVIC_ISER2 |= (1 << IRQnumber%64);
		}else if (IRQnumber >= 96 && IRQnumber < 128) {//ISER3 register
			*NVIC_ISER3 |= (1 << IRQnumber%96);
		}
	}else if (enable_disable == DISABLE)
	{//Configure ICERx (Interrupt Clear-Enable Registers)
		if (IRQnumber < 32) {//ICER0 register
			*NVIC_ICER0 |= (1 << IRQnumber);
		}else if (IRQnumber >= 32 && IRQnumber < 64) {//ICER1 register
			*NVIC_ICER1 |= (1 << IRQnumber%32);
		}else if (IRQnumber >= 64 && IRQnumber < 96) {//ICER2 register
			*NVIC_ICER2 |= (1 << IRQnumber%64);
		}else if (IRQnumber >= 96 && IRQnumber < 128) {//ICER3 register
			*NVIC_ICER3 |= (1 << IRQnumber%96);
		}
	}
}

/*****************************************************************
 * @fn		- GPIO_IRQpriorityconfig
 *
 * @brief	- This function configures interrupt priority in the NVIC
 *
 * @param[IRQNumber]	- IRQ Interrupt number
 * @param[IRQPriority]	- IRQ interrupt priority
 *****************************************************************/
void GPIO_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find the Interrupt Priority Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_priority =  (8*iprx_section) + (8-IMPLEMENTED_PR_BITS);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_priority );
}
/*****************************************************************
 * @fn		- GPIO_IRQHandling
 *
 * @brief	- This function handles the interrupt in a pin
 *
 * @param[pin_number] - Pin number
 *****************************************************************/
void GPIO_IRQhandling(uint8_t pin_number)
{//clear the EXTI Priority register corresponding to the pin number
	if (EXTI->PR & (1 << pin_number)) //clear
		EXTI->PR |= (1 << pin_number);
}
/*
 * NUCLEO-144 board special initializers
 */
/*****************************************************************
 * @fn		- GPIO_BoardLEDSInit
 *
 * @brief	- This function initializes the LEDs in the NUCLEO-144 board
 * 				Also intializes the D13 pin for the SmartGPU2
 *****************************************************************/
void GPIO_BoardLEDSInit(void){
	GPIO_Handle_t led_00;//left green led,	PB0
	GPIO_Handle_t led_07;//central blue led,PB7
	GPIO_Handle_t led_14;//right red led,	PB14
	GPIO_Handle_t pin_D13;//digital 13, 	PA5

	//memset(from string.h): Sets a block of memory to a desired value (0 in this case)
	memset(&led_00, 0, sizeof(GPIO_Handle_t));
	memset(&led_07, 0, sizeof(GPIO_Handle_t));
	memset(&led_14, 0, sizeof(GPIO_Handle_t));
	memset(&pin_D13, 0, sizeof(GPIO_Handle_t));

	//PB0
	led_00.pGPIOx = GPIOB;
	led_00.GPIO_PinConfig.GPIO_PinNumber = 0;
	led_00.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led_00.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	led_00.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	led_00.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_init(&led_00);

	//PB7
	led_07.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_init(&led_07);

	//PB14
	led_14.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_init(&led_14);

	//PA5
	pin_D13.pGPIOx = GPIOA;
	pin_D13.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIO_init(&pin_D13);
}
/*****************************************************************
 * @fn		- GPIO_BoardButtonInit
 *
 * @brief	- This function initializes the User Button in the NUCLEO-144 board
 *****************************************************************/
void GPIO_BoardButtonInit(void){
	GPIO_Handle_t board_button;
	memset(&board_button, 0, sizeof(GPIO_Handle_t));

	board_button.pGPIOx = GPIOC;
	board_button.GPIO_PinConfig.GPIO_PinNumber = 13;
	board_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	board_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	//Not an output board_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	board_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIO_init(&board_button);
	//IRQ configuration
	GPIO_IRQpriorityconfig(IRQ_VPOS_EXTI15_10, 15);
	GPIO_IRQconfig(IRQ_VPOS_EXTI15_10, ENABLE);
}

/*
 * SmartGPU2/arduino special functions
 */

void delay(uint32_t value) {
	for (uint32_t i = 0; i < (value * 2500); ++i);
}

uint8_t delay_debounce(enum ButtonStates button_state) {
    if (GPIO_readpin(GPIOC, 13)){                      /* if pressed     */
        if (button_state == PRESS){
            button_state = DOWN;
        }
        if (button_state == UP){
            delay(100);
            if (GPIO_readpin(GPIOC, 13) == 1){
                button_state = PRESS;
            }
        }
    } else {                                 /* if not pressed */
        if (button_state == RELEASE){
            button_state = UP;
        }
        if (button_state == DOWN){
            if (GPIO_readpin(GPIOC, 13) == 0){
               delay(100);
                if (GPIO_readpin(GPIOC, 13) == 0){
                    button_state = RELEASE;
                }
            }
        }
    }
    return button_state;
}

void digitalWrite(uint8_t pin_number, uint8_t value){
	GPIO_writepin(GPIOA, pin_number, value);

	//to visualize it, watch it from the blue led on-board
	GPIO_writepin(GPIOB, 7, value);
}




