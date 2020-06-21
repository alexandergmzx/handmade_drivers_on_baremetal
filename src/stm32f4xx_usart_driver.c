/*
 * stm32f4xx_usart_driver.c
 *
 *  Created on: 12 abr. xx
 *      Author: Alexander Gomez
 */

#include "../inc/stm32f4xx_usart_driver.h"

/****************************************************************************
 * @fn - USART_PeriClockControl
 * @brief - This function enables or disables peripheral
 * 			clock for the given USART port
 * @param[*pUSARTx] - Pointer to the Register Definition of the Specified USART
 * @param[enable_disable] - Macro to ENABLE or DISABLE the Clock (EnorDi)
 ****************************************************************************/
void USART_clk(USART_RegDef_t *pUSARTx, uint8_t enable_disable)
{
	if (enable_disable == ENABLE) {
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}else if (pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}else if (pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}else if (pUSARTx == UART9)
		{
			UART9_PCLK_EN();
		}else if (pUSARTx == UART10)
		{
			UART10_PCLK_EN();
		}
	}else if(enable_disable == DISABLE){
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}else if (pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		}else if (pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}else if (pUSARTx == UART9)
		{
			UART9_PCLK_DI();
		}else if (pUSARTx == UART10)
		{
			UART10_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn		- USART_init
 *
 * @brief	- This function initializes the given USART pin structure
 *
 * @param[pUSARTHandle] - Pointer to USART Handle structure
 *****************************************************************/
void USART_init(USART_Handle_t *pUSARTHandle){
	//START THE CLOCK, DONT LEAVE IT UN-INITIALIZED
	USART_clk(pUSARTHandle->pUSARTx, ENABLE);

	//Temporary variable
	uint32_t tempreg = 0;
/******************************** Configuration of CR1******************************************/
	//1. Set the mode in the TE and RE bits on the CR1
	//Enable USART Tx and Rx blocks according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{//Enable the Receiver bit field
		tempreg |= ( 1 << USART_CR1_RE );
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{//Enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{//Enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}
	//2. Set the parity bit with th parity bits at CR1
    //Configuration of Parity Control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{//Enable the Parity Control
		tempreg |= ( 1 << USART_CR1_PCE);
		//EVEN Parity is Selected by default (0)
	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{//Enable the Parity Control
	    tempreg |= ( 1 << USART_CR1_PCE);
	    //Select ODD Parity
	    tempreg |= ( 1 << USART_CR1_PS);
	}

	//3. Set the word length with the M bit at CR1
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

	//extra. set the oversampling mode over16=0, over8=1
	tempreg |= pUSARTHandle->USART_Config.USART_Oversampling << USART_CR1_OVER8;

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;
/******************************** Configuration of CR2******************************************/
	tempreg = 0;

	//4. Set the stop bits
	//Configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;
/******************************** Configuration of CR3******************************************/
	tempreg = 0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{//Enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{//Enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{//Enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
	}
	//Program the CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg;
/******************************** Configuration of BRR(Baudrate register)******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/*****************************************************************
 * @fn		- USART_deinit
 *
 * @brief	- This function de-initialize the given USART peripheral
 *
 * @param[pUSARTx] - Base address of the USART peripheral
 *****************************************************************/
void USART_deinit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}else if (pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}else if (pUSARTx == UART7)
	{
		UART7_REG_RESET();
	}else if (pUSARTx == UART8)
	{
		UART8_REG_RESET();
	}else if (pUSARTx == UART9)
	{
		UART9_REG_RESET();
	}else if (pUSARTx == UART10)
	{
		UART10_REG_RESET();
	}
}

/*
 * Data Transmission and Reception (communication without interrupts)
 * */
/*****************************************************************
 * @fn		- USART_transmit
 *
 * @brief	- This function sends data over USART peripheral
 *
 * @param[pUSARTHandle] - Base address of the USART peripheral
 * @param[pTxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *****************************************************************/
void USART_transmit(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len){
	uint16_t *pdata;
	//Loop over until len number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++){
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TXE));
         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_WORDLEN_9BITS)
		{//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{//No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer+=2;//increment pTxBuffer twice
			}else
			{//Parity bit is used in this transfer . so , 8bits of user data will be sent
				pTxBuffer++;//The 9th bit will be replaced by parity bit by the hardware
			}
		}
		else{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			pTxBuffer++;//increment the buffer address
		}
	}
	//Wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TC));
}
/*****************************************************************
 * @fn		- USART_receive
 *
 * @brief	- This function receives data over USART
 * 			  peripheral
 *
 * @param[pUSARTHandle] - Base address of the USART peripheral
 * @param[pRxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *****************************************************************/
void USART_receive(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len){
	   //Loop over until "len" number of bytes are transferred
		for(uint32_t i = 0 ; i < len; i++){
			//Wait until RXNE flag is set in the SR
			while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_RXNE));

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

					//Now increment the pRxBuffer two times
					pRxBuffer+=2;
				}else{//Parity is used, so, 8bits will be of user data and 1 bit is parity
					*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					//Increment the pRxBuffer
					pRxBuffer++;
				}
			}
			else{//We are going to receive 8bit data in a frame
				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data
					//read 8 bits from DR
					*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);;
				}else{//Parity is used, so , 7 bits will be of user data and 1 bit is parity
					//read only 7 bits , hence mask the DR with 0X7F
					*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
				}
				//increment the pRxBuffer
				pRxBuffer++;
			}
		}
}
/*
 * Data Transmission and Reception with interrupts
 * */
/*****************************************************************
 * @fn		- USART_txIT
 *
 * @brief	- This function sends data over USART
 * 			  peripheral in Interrupt mode
 *
 * @param[pUSARTHandle] - Pointer to USART Handle structure
 * @param[pTxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *
 * @return	- Tx State
 *****************************************************************/
uint8_t USART_txIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)	{
		//1. Save the Tx buffer address and Data Frame length in global variables(Handle structure)
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = len;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;
}
/*****************************************************************
 * @fn		- USART_rxIT
 *
 * @brief	- This function receives data over USART
 * 			  peripheral in Interrupt mode
 *
 * @param[pUSARTHandle] - Pointer to USART Handle structure
 * @param[pRxBuffer] - Transmit buffer
 * @param[len] - Length of transmit buffer
 *
 * @return	- Rx State
 *****************************************************************/
uint8_t USART_rxIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX){
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}
/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn		- USART_IRQConfig
 *
 * @brief	- This function Sets/clears an interrupt in the NVIC
 *
 * @param[IRQnumber] 		- IRQ Interrupt number
 * @param[enable_disable]	- Macro: Enable/Disable
 *****************************************************************/
void USART_IRQconfig(uint8_t IRQnumber, uint8_t enable_disable)
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
 * @fn		- USART_IRQpriorityconfig
 *
 * @brief	- This function configures interrupt priority in the NVIC
 *
 * @param[IRQNumber]	- IRQ Interrupt number
 * @param[IRQPriority]	- IRQ interrupt priority
 *****************************************************************/
void USART_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find the Interrupt Priority Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_priority =  (8*iprx_section) + (8-IMPLEMENTED_PR_BITS);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_priority );
}
/*****************************************************************
 * @fn		- USART_IRQHandling
 *
 * @brief	- This function handles interrupts
 *
 * @param[pUSARTHandle] - Handle structure
 *****************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle){
	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;
/**************************************** Check for TC flag ********************************************/
	//Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	//Check the state of TCIE bit in the CR
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2){//Transmission Complete interrupt!
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen ){
				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				//Clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}
/*************************Check for TXE flag ********************************************/
	//Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);
	//Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2){//this interrupt is because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0){
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//No parity is used in this transfer , so, 9bits of user data will be sent
						pUSARTHandle->pTxBuffer+=2;
						pUSARTHandle->TxLen-=2;
					}else{//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}else{//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}
			}
			if (pUSARTHandle->TxLen == 0){//TxLen is zero
				//Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}
/*************************Check for RXNE flag ********************************************/
	//Check the state of RXNE bit in SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	//Check the state of RXNEIE bit in CR
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){//RXE is set so send data
			if(pUSARTHandle->RxLen > 0){
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					//We are going to receive 9bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//No parity is used. so, all 9bits will be of user data
						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						pUSARTHandle->pRxBuffer+=2;
						pUSARTHandle->RxLen-=2;
					}else{//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen--;
					}

				}else{//We are going to receive 8bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						*(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}else{//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						*(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}

			if(! pUSARTHandle->RxLen ){
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}
/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5
	//Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);
	//Check the state of CTSE bit in CR
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	//Check the state of CTSIE bit in CR (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3){//this interrupt is because of cts
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}
/*************************Check for IDLE detection flag ********************************************/
	//Check the state of IDLE bit in SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);
	//Check the state of IDLEIE bit in CR
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2){//this interrupt is because of idle
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}
/*************************Check for OverRun detection flag ********************************************/
	//Check the status of ORE flag in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
	//Check the status of RXNEIE  bit in the CR1 // ORE is handled by RXNEIE :)
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1  && temp2){//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		//this interrupt is because of Overrun error
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_ORE);
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}
/*************************Check for Parity Error flag ********************************************/
	//Check the status of PE flag in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_PE;
	//Check the status of PEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_PEIE;

	if(temp1  && temp2){//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		//this interrupt is because of Overrun error
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_PE);
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}
/*************************Check for Error Flag ********************************************/
//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.
	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2){
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ANY);
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE)){
			/*	This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register). */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}
		if(temp1 & ( 1 << USART_SR_NF)){
			/* 	This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register). */
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}
		if(temp1 & ( 1 << USART_SR_ORE)){
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}
/*******************************************************************
 * @fn 		- USART_SetBaudRate
 *
 * @brief	- This function sets U(S)ART Baudrate
 *
 * @param[pUSARTx]	- Base address of the U(S)ART peripheral
 * @param[BaudRate]	- Baud rate value
 ********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;
	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg = 0;
	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6 || pUSARTx == UART9 || pUSARTx == UART10)
	{//USART1, USART6, UART9 and UART10 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
	}else{
	   PCLKx = RCC_GetPCLK1Value();
	}
	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else{
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}
	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{//OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
	}else
	{//over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}
	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 * Other Peripheral Control APIs
 */
/*****************************************************************
 * @fn		- USART_peripheral_control
 *
 * @brief	- This function sets USART peripheral control
 *
 * @param[pUSARTx] - Base address of the USART peripheral
 * @param[enable_disable] - Enable or Disable command
 *****************************************************************/
void USART_peripheral_control(USART_RegDef_t *pUSARTx, uint8_t enable_disable){
	if (enable_disable == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else if (enable_disable == DISABLE) {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}
/*****************************************************************
 * @fn		- USART_GetFlagStatus
 *
 * @brief	- This function returns if bit in register is
 * 			  set or not
 *
 * @param[pUSARTx] - Base address of the USART peripheral
 * @param[flag] - Name of flag
 *
 * @return	- Flag status (True/False)
 *****************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag){
	uint8_t tempable = (1 << flag);
	if(pUSARTx->SR & (1 << flag)){
		tempable = 1;
	}else  tempable = 0;
	return tempable;
}

/*****************************************************************
 * @fn		- USART_ClearFlag
 *
 * @brief	- This function clears the flag in the Status register
 *
 * @param[pUSARTx] 	- Base address of the USART peripheral
 * @param[flag] 	- Name of flag
 *****************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flag){
	pUSARTx->SR &= ~(1 << flag);
}

/*****************************************************************
 * @fn		- USART_app_event_callback
 *
 * @brief	- Application event callback function
 *
 * @param[pUSARTHandle] - Handle structure
 * @param[event]	  - Application event
 *
 * @Note	- This is a weak implementation, override it
 * 			  for your aplication
 *****************************************************************/
//__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
//{
//	//This is a weak implementation, the app can override this implementation
//}

/*
 * SmartGPU2 special functions
 */
/* USART Handle */
USART_Handle_t USART6Handle;

/* Indicates reception completion */
volatile uint8_t rxComplete = RESET;

#define USART6_AF 		(8U)
#define USART6_TX 		(14U)
#define USART6_RX 		(9U)

void USART6_GPIO_init(void){
	GPIO_Handle_t USART6pins;
	//memset(from string.h): Sets a block of memory to a desired value (0 in this case)
	memset(&USART6pins, 0, sizeof(GPIO_Handle_t));

	USART6pins.pGPIOx = GPIOG;
	USART6pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	USART6pins.GPIO_PinConfig.GPIO_PinAltFunMode = USART6_AF;
	USART6pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART6pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART6pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	//TX
	USART6pins.GPIO_PinConfig.GPIO_PinNumber = USART6_TX;
	GPIO_init(&USART6pins);
	//RX
	USART6pins.GPIO_PinConfig.GPIO_PinNumber = USART6_RX;
	GPIO_init(&USART6pins);
}

void USART6_initUSART(void){
	extern USART_Handle_t USART6Handle;
	//memset(from string.h): Sets a block of memory to a desired value (0 in this case)
	memset(&USART6Handle, 0, sizeof(USART6Handle));

	USART6Handle.pUSARTx = USART6;
	USART6Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART6Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART6Handle.USART_Config.USART_Oversampling = 1;
	USART6Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART6Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART6Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART6Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_init(&USART6Handle);

    USART_IRQconfig(IRQ_VPOS_USART6, ENABLE);
    USART_IRQpriorityconfig(IRQ_VPOS_USART6, 1);
	//This Function initializes the USART6 standard pins
	USART6_GPIO_init();
	USART_peripheral_control(USART6, ENABLE);
}

void put_char_usart(uint8_t data){
	while(USART_txIT(&USART6Handle, &data, 1) != USART_READY);
}
uint8_t get_char_usart(void){
	static uint8_t rx_char = 0;
	while(USART_rxIT(&USART6Handle, &rx_char, 1) != USART_READY);

	while(rxComplete != SET);
	rxComplete = RESET;

	return rx_char;
}
void setUsartBaud(uint32_t newBaud){
	USART_peripheral_control(USART6, DISABLE);
	USART6Handle.USART_Config.USART_Baud = newBaud;
	USART_init(&USART6Handle);
	USART_peripheral_control(USART6, ENABLE);
}

void USART6_IRQHandler(void)
{
	USART_IRQHandling(&USART6Handle);
}


void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
   if(AppEvent == USART_EVENT_RX_CMPLT){
	   rxComplete = SET;
//	   printf(":\tRx Complete\n");

   }else if (AppEvent == USART_EVENT_TX_CMPLT){
//	   printf(":\tTx Complete\n");

   }else if (AppEvent == USART_EVENT_IDLE) {
//	   printf("Usart Idle State\n");

   }else if (AppEvent == USART_ERR_ANY) {
	   printf("Error sutil\n");
   }
}

