/*
 * stm32f413xx_spi_driver.c
 *
 *  Created on: 28 mar. 2020
 *      Author: Alexander Gomez
 */

#include "../inc/stm32f413xx_spi_driver.h"

//Interrupt helper private functions Declarations
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle);

/****************************************************************************
 * @fn		- SPI_PeriClockControl
 * @brief	- This function enables or disables peripheral
 * 			clock for the given SPI
 * @param[pSPIx] - Pointer to the Register Definition of the Specified SPI
 * @param[enable_disable] - Macro to ENABLE or DISABLE the Clock (EnorDi)
 ****************************************************************************/
void SPI_clk(SPI_RegDef_t *pSPIx, uint8_t enable_disable){
	if (enable_disable == ENABLE) {
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}else if (pSPIx == SPI5)
			{
				SPI5_PCLK_EN();
			}
		}else if(enable_disable == DISABLE){
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}else if (pSPIx == SPI5)
			{
				SPI5_PCLK_DI();
			}
		}
}

/*
 * Init and De-Init (Functions to de/initialize the given SPI structure)
 */
/*****************************************************************
 * @fn		- SPI_Init
 *
 * @brief	- This function initialize SPI peripherals
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 *****************************************************************/
void SPI_init(SPI_Handle_t *pSPIHandle){
	//START THE CLOCK, DONT LEAVE IT UN-INITIALIZED
	SPI_clk(pSPIHandle->pSPIx, ENABLE);
	//First configure the CR1 with this temporal register(temp0)
	uint32_t temp0 = 0;
	//1. Configure the master/slave Mode
	temp0 |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
	//2. Configure the xxplex Bus Config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{//BI-DIrectional mode should be cleared
		temp0 &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{//BI-DIrectional mode should be set
		temp0 |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{//Set RX-Only register and BI-DIrectional mode should be cleared
		temp0 |= (1 << SPI_CR1_RXONLY);
		temp0 &= ~(1 << SPI_CR1_BIDIMODE);
	}
	//3. Configure the Data Frame Format
	temp0 |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//4. Configure the Clock POLarity
	temp0 |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//5. Configure the Clock PHAse
	temp0 |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//6. Configure the Serial Clock Speed (Baud Rate)
	temp0 |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	temp0 |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp0;

}
/*****************************************************************
 * @fn		- SPI_deinit
 *
 * @brief	- This function de-initialize SPI peripherals
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 *****************************************************************/
void SPI_deinit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
}
/*****************************************************************
 * @fn		- SPI_GetFlagStatus
 *
 * @brief	- This function returns if bit in register is
 * 			  set or not
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[flag] - Name of flag
 *
 * @return	- Flag status (True/False)
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag){
	return(pSPIx->SR & (1 << flag));
}
/*
 * Data Transmission and Reception (communication without interrupts)
 * */
/*****************************************************************
 * @fn		- SPI_transmit
 *
 * @brief	- This function sends data over SPI peripheral
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[pTxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *****************************************************************/
void SPI_transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	//SPI_peripheral_control(pSPIx,ENABLE);
	while(len > 0)
	{
		//1. Wait until TXE is available
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == RESET);
		//2. Check the Data Frame Format bit in CR1
		if (pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{//16 bit mode
			//2.2- Load the data to the SPI DataRegister
			pSPIx->DR = *((uint16_t*)pTxBuffer);//loads 2 bytes
			len-=2;//decreases twice the length
			(uint16_t*)pTxBuffer++;//shift the pointer to continue
		} else
		{//8 bit mode
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;//shift the pointer to continue
		}
	}
	while( SPI_GetFlagStatus(pSPIx,SPI_SR_BSY) );
	//SPI_peripheral_control(pSPIx,DISABLE);
}
/*****************************************************************
 * @fn		- SPI_receive
 *
 * @brief	- This function receives data over SPI
 * 			  peripheral
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[pRxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *****************************************************************/
void SPI_receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	//SPI_peripheral_control(pSPIx,ENABLE);
	while(len > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_RXNE)  == (uint8_t)RESET );
		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{//16 bit DFF
			//1. load the data from DR to Rxbuffer address
			 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
			len-=2;
			(uint16_t*)pRxBuffer++;
		}else
		{//8 bit DFF
			*(pRxBuffer) = pSPIx->DR ;
			len--;
			pRxBuffer++;
		}
	}
	while( SPI_GetFlagStatus(pSPIx,SPI_SR_BSY) );
	//SPI_peripheral_control(pSPIx,DISABLE);
}

/*
 * Data Transmission and Reception with interrupts
 * */
/*****************************************************************
 * @fn		- SPI_txIT
 *
 * @brief	- This function sends data over SPI
 * 			  peripheral in Interrupt mode
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 * @param[pTxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *
 * @return	- Tx State
 *****************************************************************/
uint8_t SPI_txIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->Txstate;
	if(state != SPI_BUSY_IN_TX){
		//1. Save the Tx buffer address and Data Frame length in global variables(Handle structure)
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->Txlen = len;
		//2. Mark the SPI state as busy so the SPI is not taken over by other function
		pSPIHandle->Txstate = SPI_BUSY_IN_TX;
		//3. Enable the TXE IE (Transmission buffer Empty? Interrupt Enable) control bit to get the interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
/*****************************************************************
 * @fn		- SPI_rxIT
 *
 * @brief	- This function receives data over SPI
 * 			  peripheral in Interrupt mode
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 * @param[pRxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *
 * @return	- Rx State
 *****************************************************************/
uint8_t SPI_rxIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->Rxstate;
	if(state != SPI_BUSY_IN_RX){
		//1. Save the Tx buffer address and Data Frame length in global variables(Handle structure)
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->Rxlen = len;
		//2. Mark the SPI state as busy so the SPI is not taken over by other function
		pSPIHandle->Rxstate = SPI_BUSY_IN_RX;
		//3. Enable the RXNE IE (Reception buffer NotEmpty? Interrupt Enable)
		//control bit to get the interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn		- SPI_IRQconfig
 *
 * @brief	- This function configures interrupt
 *
 * @param[IRQnumber] - IRQ Interrupt number
 * @param[enable_disable] - Macro: Enable/Disable
 *****************************************************************/
void SPI_IRQconfig(uint8_t IRQnumber, uint8_t enable_disable)
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
 * @fn		- SPI_IRQPriorityConfig
 *
 * @brief	- This function configures interrupt priority
 *
 * @param[IRQNumber] - IRQ Interrupt number
 * @param[IRQPriority] - IRQ interrupt priority
 *****************************************************************/
void SPI_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find the Interrupt Priority Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_priority =  (8*iprx_section) + (8-IMPLEMENTED_PR_BITS);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_priority );
}
/*****************************************************************
 * @fn		- SPI_IRQHandling
 *
 * @brief	- This function handles interrupts
 *
 * @param[pSPIHandle] - Handle structure
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{//1. Check the Status Register toget the event that raised the interrupt
	//TX ,RX or ERR flag
	uint8_t temp1, temp2;
	//Check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{//handle TXE
		SPI_TXE_interrupt_handle(pSPIHandle);
	}
	//Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{//handle TXE
		SPI_RXNE_interrupt_handle(pSPIHandle);
	}
	//Check for OVR ERR(Overrun error)
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{//handle TXE
		SPI_OVR_interrupt_handle(pSPIHandle);
	}
}

/*
 * Other Peripheral Control APIs
 */
/*****************************************************************
 * @fn		- SPI_peripheral_control
 *
 * @brief	- This function sets SPI peripheral control
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[enable_disable] - Enable or Disable command
 *****************************************************************/
void SPI_peripheral_control(SPI_RegDef_t *pSPIx, uint8_t enable_disable){
	if (enable_disable == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else if (enable_disable == DISABLE) {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/*****************************************************************
 * @fn		- SPI_SSIConfig
 *
 * @brief	- This function sets SSI register
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[enable_disable] - Enable or Disable command
 *****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable_disable){
	if (enable_disable == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else if (enable_disable == DISABLE) {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/*****************************************************************
 * @fn		- SPI_SSOEConfig
 *
 * @brief	- This function sets SSEO register
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 * @param[enable_disable] - Enable or Disable command
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable_disable){
	if (enable_disable == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else if (enable_disable == DISABLE) {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
/*****************************************************************
 * @fn		- SPI_CloseTransmission
 *
 * @brief	- This function close SPI transmission
 *
 * @param[pSPIHandle] - Handle structure
 *****************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//Deactivate the TXEIE
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->Txlen = 0;
	pSPIHandle->Txstate = SPI_READY;
}
/*****************************************************************
 * @fn		- SPI_CloseReception
 *
 * @brief	- This function close SPI reception
 *
 * @param[pSPIHandle] - Handle structure
 *****************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//Deactivate the RXNEIE
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->Rxlen = 0;
	pSPIHandle->Rxstate = SPI_READY;
}
/*****************************************************************
 * @fn		- SPI_ClearOVRFlag
 *
 * @brief	- This function clears OVR flag
 *
 * @param[pSPIx] - Base address of the SPI peripheral
 *****************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*****************************************************************
 *     Interrupt helper private(static) functions
 *****************************************************************/
/*****************************************************************
 * @fn		- SPI_app_event_callback
 *
 * @brief	- Application event callback function
 *
 * @param[pSPIHandle] - Handle structure
 * @param[event]	  - Application event
 *
 * @Note	- This is a weak implementation, override it
 * 			  for your aplication
 *****************************************************************/
__attribute__((weak)) void SPI_app_event_callback(SPI_Handle_t *pSPIHandle,uint8_t event)
{
	//This is a weak implementation, the app can override this implementation
}

/*****************************************************************
 * @fn		- SPI_TXE_interrupt_handle
 *
 * @brief	- This function handles TXE in interrupt mode
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 *****************************************************************/
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the Data Frame Format bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{//16 bit mode
		//2.2- Load the data to the SPI DataRegister
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);//loads 2 bytes
		pSPIHandle->Txlen-=2;//decreases twice the length
		((uint16_t*)(pSPIHandle->pTxBuffer)++);//shift the pointer to continue
	} else
	{//8 bit mode
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->Txlen--;
		pSPIHandle->pTxBuffer++;//shift the pointer to continue
	}

	if (!(pSPIHandle->Txlen))
	{//The transmission is over (len = 0)
		//Deactivate the TX
		SPI_CloseTransmission(pSPIHandle);

		SPI_app_event_callback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}
/*****************************************************************
 * @fn		- SPI_RXNE_interrupt_handle
 *
 * @brief	- This function handles RXNE in interrupt mode
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 *****************************************************************/
static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{//16 bit DFF
		//1. load the data from DR to Rxbuffer address
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
		 pSPIHandle->Rxlen-=2;
		 pSPIHandle->pRxBuffer++;
		 pSPIHandle->pRxBuffer++;
	}else
	{//8 bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
		pSPIHandle->Rxlen--;
		pSPIHandle->pRxBuffer++;
	}
	if (!(pSPIHandle->Rxlen))
	{//The reception is over (len = 0)
		//Deactivate the RX
		SPI_CloseReception(pSPIHandle);

		SPI_app_event_callback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
/*****************************************************************
 * @fn		- SPI_OVR_interrupt_handle
 *
 * @brief	- This function handles OVR_ERR in
 * 		          interrupt mode
 *
 * @param[pSPIHandle] - Pointer to SPI Handle structure
 *****************************************************************/
static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//1. Clear the OVR flag !ALWAYS READ THE REFERENCE MANUAL! don't just pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_ERRIE);
	if (pSPIHandle->Txstate != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Inform to the app (via callback)
	SPI_app_event_callback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
