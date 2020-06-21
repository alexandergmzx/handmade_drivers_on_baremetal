/*
 * stm32f4xx_usart_driver.h
 *
 *  Created on: 12 abr. 2020
 *      Author: Alexander Gomez
 */

#ifndef INC_STM32F4XX_USART_DRIVER_H_
#define INC_STM32F4XX_USART_DRIVER_H_

#include "stm32f413xx.h"

/*
 * Configuration structure for U(S)ARTx peripheral
 */
typedef struct
{
	uint8_t  USART_Mode;			/*< Possible USART modes from	@SPI_CLOCK_PHASE				>*/
	uint8_t  USART_Oversampling;	/*< Possible USART modes from	@SPI_CLOCK_PHASE				>*/
	uint32_t USART_Baud;			/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
	uint8_t  USART_NoOfStopBits;	/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
	uint8_t	 USART_WordLength;		/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
	uint8_t	 USART_ParityControl;	/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
	uint8_t  USART_HWFlowControl;	/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
}USART_Config_t;


/*
 * Handle structure for  U(S)ARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;	/* This holds the base address of USARTx(x:0,1,2) peripheral */
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;		/* Stores application Tx buffer address */
	uint8_t *pRxBuffer;		/* Stores application Rx buffer address */
	uint32_t TxLen;			/* Stores Tx length			*/
	uint32_t RxLen;			/* Stores Rx length			*/
	uint8_t TxBusyState;		/* Transmission is in  busy state	*/
	uint8_t RxBusyState;		/* Receiving is in  busy state	   	*/
}USART_Handle_t;

/*
 * @USART_Mode
 * Possible options for USART_Mode
 */
enum usart_modes{
	USART_MODE_ONLY_TX,
	USART_MODE_ONLY_RX,
	USART_MODE_TXRX,
};

/*
 * @USART_Baud
 * Possible options for USART_Baud
 */
enum usart_std_baud{
	USART_STD_BAUD_1200=1200,
	USART_STD_BAUD_2400=2400,
	USART_STD_BAUD_9600=9600,
	USART_STD_BAUD_19200=19200,
	USART_STD_BAUD_38400=38400,
	USART_STD_BAUD_57600=57600,
	USART_STD_BAUD_115200=115200,
	USART_STD_BAUD_230400=230400,
	USART_STD_BAUD_460800=460800,
	USART_STD_BAUD_921600=921600,
	USART_STD_BAUD_2M=2000000,
	SUART_STD_BAUD_3M=3000000,
};

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
enum usart_stop_bits{
	USART_STOPBITS_1,
	USART_STOPBITS_0_5,
	USART_STOPBITS_2,
	USART_STOPBITS_1_5,
};
/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
enum usart_word_length{
	USART_WORDLEN_8BITS,
	USART_WORDLEN_9BITS,
};
/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
enum usart_parity_mode{
	USART_PARITY_DISABLE,
	USART_PARITY_EN_EVEN,
	USART_PARITY_EN_ODD,
};
/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
enum usart_hw_flow_mode{
	USART_HW_FLOW_CTRL_NONE,
	USART_HW_FLOW_CTRL_CTS,
	USART_HW_FLOW_CTRL_RTS,
	USART_HW_FLOW_CTRL_CTS_RTS,
};

/*
 * USART Flags
 */
#define USART_FLAG_TXE 		( 1 << USART_SR_TXE  )
#define USART_FLAG_RXNE 	( 1 << USART_SR_RXNE )
#define USART_FLAG_TC 		( 1 << USART_SR_TC   )


/*
 * @USART_APPLICATION_STATES
 * USART application states
 */
enum usart_application_states{
	USART_READY,
	USART_BUSY_IN_RX,
	USART_BUSY_IN_TX,
};

/*
 * @USART_APPLICATION_EVENTS
 * Possible USART Application events
 */
enum usart_app_events{
	USART_EVENT_TX_CMPLT,
	USART_EVENT_RX_CMPLT,
	USART_EVENT_IDLE,
	USART_EVENT_CTS,
	USART_EVENT_PE,
	USART_ERR_FE,
	USART_ERR_NE,
	USART_ERR_ORE,
	USART_ERR_ANY,
};

/*******************************************************************************
 * 				APIs supported by this driver
 * For more information about the functionality check the function definitions
 * ***************************************************************************** */

//periphertal clock control
void USART_clk(USART_RegDef_t *pUSARTx, uint8_t enable_disable);

//Function to de/initialize the given USART port and Pin
void USART_init(USART_Handle_t *pUSARTHandle);//Needs all the function Handle
void USART_deinit(USART_RegDef_t *pUSARTx);//Just needs the reset register
/*
 * Data Transmission and Reception
 * */
void USART_transmit(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_receive(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * Data Transmission and Reception with interrupts
 * */
uint8_t USART_txIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_rxIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

//functions for interrupt handling
void USART_IRQconfig(uint8_t IRQnumber,uint8_t enable_disable);
void USART_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_peripheral_control(USART_RegDef_t *pUSARTx, uint8_t enable_disable);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flag);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);//Tx/Rx baud = clockFreq(f_ck) / (8 * (2-OVER8) * USARTDIV)

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);

/*
 * SmartGPU2 special functions
 */
void USART6_GPIO_init(void);
void USART6_initUSART(void);

void put_char_usart(uint8_t data);
uint8_t get_char_usart(void);
void setUsartBaud(uint32_t newBaud);


#endif /* INC_STM32F4XX_USART_DRIVER_H_ */
