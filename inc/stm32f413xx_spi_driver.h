/*
 * stm32f413xx_spi_driver.h
 *
 *  Created on: 28 mar. 2020
 *      Author: Alexander Gomez
 */

#ifndef INC_STM32F413XX_SPI_DRIVER_H_
#define INC_STM32F413XX_SPI_DRIVER_H_

#include "stm32f413xx.h"

//Structure to hold the Configuration of the SPIx peripheral
typedef struct
{
	uint8_t SPI_DeviceMode; /*< Mastr/Slave modes 				@SPI_DEVICE_MODES				>*/
	uint8_t SPI_BusConfig;	/*< Possible bus configs from		@SPI_BUS_CONFIGURATIONS			>*/
	uint8_t SPI_DFF;		/*< Possible DataFramFormats from	@SPI_DATA_FRAME_FORMATS			>*/
	uint8_t SPI_CPOL;		/*< Possible clk polarities from	@SPI_CLOCK_POLARITIES			>*/
	uint8_t SPI_CPHA;		/*< Possible clk phases from		@SPI_CLOCK_PHASE				>*/
	uint8_t SPI_SSM;		/*< En/disable sw slave management 	@SPI_SOFTWARE_SLAVE_MANAGEMENT	>*/
	uint8_t SPI_SclkSpeed;	/*< Possible speed divisions from	@SPI_SYSTEMCLOCK_SPEED			>*/
}SPI_Config_t;

//Structure to handle the base adddress of the peripheral and its config of the SPI
typedef struct
{//pointer to hold the base address of the SPI Peripheral
	SPI_RegDef_t *pSPIx;//SPI base address
	SPI_Config_t SPI_Config;//Configuration of the SPI registers
	uint8_t 	 *pTxBuffer;/* !< To store the app. Tx buffer address 	>*/
	uint8_t 	 *pRxBuffer;/* !< To store the app. Rx buffer address 	>*/
	uint8_t 	 Txstate;	/* !< To store Tx state 					>*/
	uint8_t 	 Rxstate;	/* !< To store Rx state 					>*/
	uint32_t 	 Txlen;		/* !< To store Tx len 						>*/
	uint32_t 	 Rxlen;		/* !< To store Tx len 						>*/
}SPI_Handle_t;

/*
 * @SPI_DEVICE_MODES
 * Master/Slave modes
 */
enum spi_device_modes{
	SPI_DEVICE_MODE_SLAVE,
	SPI_DEVICE_MODE_MASTER,
};
/*
 * @SPI_BUS_CONFIGURATIONS
 * Full Duplex. Half Duplex or Simplex configuration
 * */
enum spi_bus_configurations{
	SPI_BUS_CONFIG_FD=1,
	SPI_BUS_CONFIG_HD,
	//SPI_BUS_CONFIG_SIMPLEX_TXONLY, //just cut the cable
	SPI_BUS_CONFIG_SIMPLEX_RXONLY,
};
/*
 * @SPI_DATA_FRAME_FORMATS
 * 8-bit or 16-bit Frame Format
 * */
enum spi_dataframe_formats{
	SPI_DFF_8BITS,
	SPI_DFF_16BITS,
};
/*
 * @SPI_CLOCK_POLARITIES
 * High or Low
 * */
enum spi_clock_polarities{
	SPI_CPOL_LOW,
	SPI_CPOL_HIGH,
};
/* @SPI_CLOCK_PHASE
 * Leading or Trailing Edge
 * */
enum spi_clock_phase{
	SPI_CPHA_LEADING,//This means LOW in the other Library
	SPI_CPHA_TRAILING,//This means HIGH in the other Library
};
/*
 * @SPI_SOFTWARE_SLAVE_MANAGEMENT
 * */
enum spi_sw_slave{
	SPI_SSM_DI,
	SPI_SSM_EN,
};
/*
 * @SPI_SYSTEMCLOCK_SPEED
 * Actually these are clock dividers for the baud rate cotrol
 * Serial Clock Frequency = Peripheral Clock Frequency ÷ DIV prescaler
 * */
enum spi_speeds{
	SPI_SCLK_SPEED_DIV2,
	SPI_SCLK_SPEED_DIV4,
	SPI_SCLK_SPEED_DIV8,
	SPI_SCLK_SPEED_DIV16,
	SPI_SCLK_SPEED_DIV32,
	SPI_SCLK_SPEED_DIV64,
	SPI_SCLK_SPEED_DIV128,
	SPI_SCLK_SPEED_DIV256,
};

/*
 * @SPI_APPLICATION_STATES
 * SPI application states
 */
enum spi_application_states{
	SPI_READY,
	SPI_BUSY_IN_RX,
	SPI_BUSY_IN_TX,
};
/*
 * @SPI_APPLICATION_EVENTS
 * Possible SPI Application events
 */
enum spi_application_events{
	SPI_EVENT_TX_CMPLT=1,
	SPI_EVENT_RX_CMPLT,
	SPI_EVENT_OVR_ERR,
	SPI_EVENT_CRC_ERR,
};
/*******************************************************************************
 * 				APIs supported by this driver
 * For more information about the functionality check the function definitions
 * ***************************************************************************** */
/*
 * Peripheral Clock setup
 * */
void SPI_clk(SPI_RegDef_t *pSPIx, uint8_t enable_disable);

/*
 * Init and Deinit
 * */
void SPI_init(SPI_Handle_t *pSPIHandle);//Needs all the function Handle
void SPI_deinit(SPI_RegDef_t *pSPIx);//Just needs the reset register

/*
 * Data Transmission and Reception
 * */
void SPI_transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * Data Transmission and Reception with interrupts
 * */
uint8_t SPI_txIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_rxIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 * */
//functions for interrupt handling
void SPI_IRQconfig(uint8_t IRQnumber,uint8_t enable_disable);
void SPI_IRQpriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_peripheral_control(SPI_RegDef_t *pSPIx, uint8_t enable_disable);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable_disable);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable_disable);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_app_event_callback(SPI_Handle_t *pSPIHandle,uint8_t event);

#endif /* INC_STM32F413XX_SPI_DRIVER_H_ */
