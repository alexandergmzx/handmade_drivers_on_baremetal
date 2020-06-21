/*
 * stm32f413xx.h
 *
 *  Created on: Mar 18, 2020
 *      Author: Alexander Gomez
 */

#ifndef INC_STM32F413XX_H_
#define INC_STM32F413XX_H_

#include <stdio.h>
#include <string.h>
#include <stddef.h>

//#define __vo		volatile
//#define __weak	__attribute__((weak))

#define TRUE 						(1)
#define FALSE 						(0)

#define ENABLE 						TRUE
#define DISABLE 					FALSE

#define SET 						TRUE
#define RESET 						FALSE

#define HIGH 						TRUE
#define LOW 						FALSE

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1(control register)
 */
enum SPI_CR1_bit_positions{
	SPI_CR1_CPHA,
	SPI_CR1_CPOL,
	SPI_CR1_MSTR,
	SPI_CR1_BR,
	SPI_CR1_SPE=6,
	SPI_CR1_LSBFIRST,
	SPI_CR1_SSI,
	SPI_CR1_SSM,
	SPI_CR1_RXONLY,
	SPI_CR1_DFF,
	SPI_CR1_CRCNEXT,
	SPI_CR1_CRCEN,
	SPI_CR1_BIDIOE,
	SPI_CR1_BIDIMODE,
};

/*
 * Bit position definitions SPI_CR2(control register)
 */
enum SPI_CR2_bit_positions{
	SPI_CR2_RXDMAEN,
	SPI_CR2_TXDMAEN,
	SPI_CR2_SSOE,
	SPI_CR2_FRF=4,
	SPI_CR2_ERRIE,
	SPI_CR2_RXNEIE,
	SPI_CR2_TXEIE,
};


/*
 * Bit position definitions SPI_SR(status register)
 */
enum SPI_SR_bit_positions{
	SPI_SR_RXNE,
	SPI_SR_TXE,
	SPI_SR_CHSIDE,
	SPI_SR_UDR,
	SPI_SR_CRCERR,
	SPI_SR_MODF,
	SPI_SR_OVR,
	SPI_SR_BSY,
	SPI_SR_FRE,
};

/******************************************************************************************
 *			Bit position definitions of U(S)ART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1(control register)
 */
enum USART_CR1_bit_positions{
	USART_CR1_SBK,
	USART_CR1_RWU,
	USART_CR1_RE,
	USART_CR1_TE,
	USART_CR1_IDLEIE,
	USART_CR1_RXNEIE,
	USART_CR1_TCIE,
	USART_CR1_TXEIE,
	USART_CR1_PEIE,
	USART_CR1_PS,
	USART_CR1_PCE,
	USART_CR1_WAKE,
	USART_CR1_M,
	USART_CR1_UE,
	USART_CR1_OVER8=15,
};

/*
 * Bit position definitions USART_CR2(control register)
 */
enum USART_CR2_bit_positions{
	USART_CR2_ADD,
	USART_CR2_LBDL=5,
	USART_CR2_LBDIE,
	USART_CR2_LBCL,
	USART_CR2_CPHA,
	USART_CR2_CPOL,
	USART_CR2_CLKEN,
	USART_CR2_STOP,
	USART_CR2_LINEN=14,
};

/*
 * Bit position definitions USART_CR3(control register)
 */
enum USART_CR3_bit_positions{
	USART_CR3_EIE,
	USART_CR3_IREN,
	USART_CR3_IRLP,
	USART_CR3_HDSEL,
	USART_CR3_NACK,
	USART_CR3_SCEN,
	USART_CR3_DMAR,
	USART_CR3_DMAT,
	USART_CR3_RTSE,
	USART_CR3_CTSE,
	USART_CR3_CTSIE,
	USART_CR3_ONEBIT,
};

/*
 * Bit position definitions USART_SR(status register)
 */
enum USART_SR_bit_positions{
	USART_SR_PE,
	USART_SR_FE,
	USART_SR_NF,
	USART_SR_ORE,
	USART_SR_IDLE,
	USART_SR_RXNE,
	USART_SR_TC,
	USART_SR_TXE,
	USART_SR_LBD,
	USART_SR_CTS,
};
/*************************	Processor Specific Details	************************/
/*
 * ARM Cortex M4 Processor NVIC Register Addresses
 * */
#define NVIC_ISER0					((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1					((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2					((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3					((volatile uint32_t *)0xE000E10C)

#define NVIC_ICER0					((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1					((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2					((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3					((volatile uint32_t *)0xE000E18C)

#define NVIC_IPR_BASEADDR			((volatile uint32_t *)0xE000E400)

//Number of priority bits implemented in priority register
#define IMPLEMENTED_PR_BITS			(4)

//Base addresses of SRAM & Flash memories
#define FLASH_BASEADDR				(0x00000000UL)
#define SRAM1_BASEADDR				(0x20000000UL)
#define SRAM2_BASEADDR				(SRAM1_BASEADDR + 0x00001C00UL)//112KB=0x1C00
#define ROM							(0x1FFF0000UL)
#define SRAM						SRAM1_BASEADDR

//AHB & APB bus domains
#define PERIPH_BASEADDR				(0x40000000UL)
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			(0x40010000UL)

#define AHB1PERIPH_BASEADDR			(0x40020000UL)
#define AHB2PERIPH_BASEADDR			(0x50000000UL)
#define AHB3PERIPH_BASEADDR			(0x60000000UL)

//APB (Advanced Peripheral Bus) peripheral domains
//APB1: TIMs 2-7 & 12-14, LPTIM1, RTC(Real-Time Clock) & BKP registers,
//WWDG(Window Watch Dog), IWDG(Independent Watch Dog),
//SPI/I2S 2-3, I2S(Inter-IC Sound) ext 2-3,
//USART 2-3, UART 4-5 & 7-8,
//I2C(Inter-Integrated Circuit) 1-3, I2C FMP (Fast Mode Plus) 1,
//CAN (Controller Area Network ) 1-3,
//PWR (Power controller) and DAC (Digital-to-Analog Converter)
#define TIM2_BASEADDR				APB1PERIPH_BASEADDR
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 1))
#define TIM4_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 2))
#define TIM5_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 3))
#define TIM6_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 4))
#define TIM7_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 5))
#define TIM12_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 6))
#define TIM13_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 7))
#define TIM14_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 8))
#define LPTIM1_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 9))

#define RTC_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 10))
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 11))
#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 12))

#define I2S2_EXT_BASEADDR			(APB1PERIPH_BASEADDR + (0x400U * 13))
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 14))
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 15))
#define I2S3_EXT_BASEADDR			(APB1PERIPH_BASEADDR + (0x400U * 16))

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 17))
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 18))
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 19))
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 20))

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 21))
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 22))
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 23))
#define I2CFMP1_BASEADDR			(APB1PERIPH_BASEADDR + (0x400U * 24))

#define CAN1_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 25))
#define CAN2_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 26))
#define CAN3_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 27))

#define PWR_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 28))
#define DAC_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 29))

#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 30))
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + (0x400U * 31))

//APB2: TIM 1 8 9-11, USART1-2, UART9-10, SYSCFG, SPI 1 4 5. EXTI, SAI
#define TIM1_BASEADDR				APB2PERIPH_BASEADDR
#define TIM8_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 1))

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 4))
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 5))
#define UART9_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 6))
#define UART10_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 7))

#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 8))

#define SDIO_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 11))

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 12))
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 13))

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 14))
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 15))

#define TIM9_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 16))
#define TIM10_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 17))
#define TIM11_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 18))

#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + (0x400U * 20))

#define SAI_BASEADDR				(APB2PERIPH_BASEADDR + 0x5800UL)

#define DFSDM1_BASEADDR				(APB2PERIPH_BASEADDR + 0x6000UL)
#define DFSDM2_BASEADDR				(APB2PERIPH_BASEADDR + 0x6400UL)

//AHB (Advanced High-performance Bus) peripheral domains
//AHB1: Contains GPIOs, CRC, RCC and DMA
#define GPIOA_BASEADDR				AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 1))
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 2))
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 3))
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 4))
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 5))
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 6))
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + (0x400U * 7))

#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3000UL)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800UL)

#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6000UL)
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6400UL)


//AHB2: Contains USB On-The-Go and RNG (Random Number Generator)
#define USB_OTG_BASEADDR 			AHB2PERIPH_BASEADDR
#define RNG_BASEADDR				(AHB2PERIPH_BASEADDR + 0x60800UL)

//AHB3: Contains FSMC(Flexible Static Memory Controller) and Quad-SPI(Serial Peripheral Interface)
//as well as their control registers.

#define FSMC_BASEADDR				AHB3PERIPH_BASEADDR
#define FSMC_CR_BASEADDR			(0xA0000000UL)

#define QUADSPI_BASEADDR			(0x90000000UL)
#define QUADSPI_CR_BASEADDR			(FSMC_CR_BASEADDR + 0x1000U)

/* Peripheral Register Definition  */
//GPIO registers
typedef struct{
	volatile uint32_t MODER;	//GPIO port MODE Register, 				offset:0x00
	volatile uint32_t OTYPER;	//GPIO port Output TYPE Register, 		offset:0x04
	volatile uint32_t OSPEEDR;	//GPIO port Output SPEED Register,		offset:0x08
	volatile uint32_t PUPDR;	//GPIO port PullUp-PullDown Register,	offset:0x0C
	volatile uint32_t IDR;		//GPIO port Input Data Register,		offset:0x10
	volatile uint32_t ODR;		//GPIO port Output Data Register,		offset:0x14
	volatile uint32_t BSRR;		//GPIO port Bit Set-Reset Register,		offset:0x18
	volatile uint32_t LCKR;		//GPIO port lock Register, 				offset:0x1C
	volatile uint32_t AFR[2];	//GPIO port Alt Function registers,		offset:0x20
}GPIO_RegDef_t;

//RCC registers
typedef struct{
	volatile uint32_t CR;		//offset:0x00
	volatile uint32_t PLLCFGR;	//offset:0x04
	volatile uint32_t CFGR;		//offset:0x08
	volatile uint32_t CIR;		//offset:0x0C
	volatile uint32_t AHB1RSTR;	//offset:0x10
	volatile uint32_t AHB2RSTR;	//offset:0x14
	volatile uint32_t AHB3RSTR;	//offset:0x18
	uint32_t RESERVED0;//0x1C 8

	volatile uint32_t APB1RSTR;	//offset:0x20
	volatile uint32_t APB2RSTR;	//offset:0x24
	uint32_t RESERVED1;//0x28 11
	uint32_t RESERVED2;

	volatile uint32_t AHB1ENR;	//offset:0x30
	volatile uint32_t AHB2ENR;	//offset:0x34
	volatile uint32_t AHB3ENR;	//offset:0x38
	uint32_t RESERVED3;//0x3C 16

	volatile uint32_t APB1ENR;	//offset:0x40
	volatile uint32_t APB2ENR;	//offset:0x44
	uint32_t RESERVED4;//0x48 19
	uint32_t RESERVED5;

	volatile uint32_t AHB1LPENR;//offset:0x50
	volatile uint32_t AHB2LPENR;//offset:0x54
	volatile uint32_t AHB3LPENR;//offset:0x58
	uint32_t RESERVED6;//0x5C 24

	volatile uint32_t APB1LPENR;//offset:0x60
	volatile uint32_t APB2LPENR;//offset:0x64
	uint32_t RESERVED7;//0x68 27
	uint32_t RESERVED8;

	volatile uint32_t BDCR;		//offset:0x70
	volatile uint32_t CSR;		//offset:0x74
	uint32_t RESERVED9;//0x78 31
	uint32_t RESERVED10;

	volatile uint32_t SSCFGR;	//offset:0x80
	volatile uint32_t PLLI2SCFGR;//offset:0x84
	uint32_t RESERVED11;//0x88 35

	volatile uint32_t DCKCFGR;	//offset:0x8C
	volatile uint32_t CKGATENR;	//offset:0x90
	volatile uint32_t DCKCFGR2; //offset:0x94
}RCC_RegDef_t;

//EXTI peripheral register definition structure
typedef struct{
	volatile uint32_t IMR;  	//Interrupt Mask Register,				offset: 0x00
	volatile uint32_t EMR;  	//Event Mask Register,					offset: 0x04
	volatile uint32_t RTSR; 	//Rising Trigger Selection Register,	offset: 0x08
	volatile uint32_t FTSR; 	//Falling Trigger Selection Register,	offset: 0x0C
	volatile uint32_t SWIER;	//SoftWare Interrupt Event Register,	offset: 0x10
	volatile uint32_t PR; 		//Pending (Request) Register,			offset: 0x14
}EXTI_RegDef_t;

//SYSCFG peripheral register definition structure for SYSCFG
typedef struct{
	volatile uint32_t MEMRMP;   //MEMory Re-MaP register,				offset: 0x00
	volatile uint32_t PMC;      //Peripheral Mode Config Register, 		offset: 0x04
	volatile uint32_t EXTICR[4];//EXTernal Interrupt Config Register,offset: 0x08-0x14
	uint32_t RESERVED1[2];//0x18-0x1C
	volatile uint32_t CMPCR;    //CoMPensation cell Control Register,	offset: 0x20
	uint32_t RESERVED2[2];//0x24-0x28
	volatile uint32_t CFGR;     //(SYSCFG)ConFiGuration Register, 		offset: 0x2C
} SYSCFG_RegDef_t;

//peripheral register definition structure for the Serial Peripheral Interface
typedef struct{//I2S stands for Inter-IC Sound
//CRC stands for Cyclic Redundancy Checking
	volatile uint32_t CR1;    	//Control Register 1,					offset: 0x00
	volatile uint32_t CR2;    	//Control Register 2,					offset: 0x04
	volatile uint32_t SR;     	//Status Register,						offset: 0x08
	volatile uint32_t DR;     	//Data Register,						offset: 0x0C
	volatile uint32_t CRCPR;  	//CRC Polynomial Register, 				offset: 0x10
	volatile uint32_t RXCRCR; 	//Reception CRC Register,				offset: 0x14
	volatile uint32_t TXCRCR;	//Transmission CRC Register,			offset: 0x18
	volatile uint32_t I2SCFGR;	//I2S Configuration Register,			offset: 0x1C
	volatile uint32_t I2SPR; 	//I2S Prescaler Register,				offset: 0x20
} SPI_RegDef_t;

typedef struct{
	volatile uint32_t SR;  		//Status Register,						offset: 0x00
	volatile uint32_t DR;  		//Data Register,						offset: 0x04
	volatile uint32_t BRR;		//Baud Rate Register,					offset: 0x08
	volatile uint32_t CR1;		//Control Register 1,					offset: 0x0C
	volatile uint32_t CR2;		//Control Register 2,					offset: 0x10
	volatile uint32_t CR3;		//Control Register 3,					offset: 0x14
	volatile uint32_t GPTR;		//Guard Time and Prescaler Register, 	offset: 0x18
} USART_RegDef_t;

//peripheral addresses typecasted to Register Definition
#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC							(( RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1  						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  						((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5  						((SPI_RegDef_t*)SPI5_BASEADDR)

#define USART1  					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  					((USART_RegDef_t*)USART3_BASEADDR)
#define USART6  					((USART_RegDef_t*)USART6_BASEADDR)
#define UART4  						((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  						((USART_RegDef_t*)UART5_BASEADDR)
#define UART7  						((USART_RegDef_t*)UART7_BASEADDR)
#define UART8  						((USART_RegDef_t*)UART8_BASEADDR)
#define UART9  						((USART_RegDef_t*)UART9_BASEADDR)
#define UART10  				   ((USART_RegDef_t*)UART10_BASEADDR)

/*
#define I2C1  						((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  						((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  						((I2C_RegDef_t*)I2C3_BASEADDR)
*/
/*RCClock enables for buses*/
/***********************************************************************************
 *APB1:	TIMs 2-7 & 12-14, LPTIM1, RTC(Real-Time Clock) & BKP registers,
 * 		WWDG(Window Watch Dog), IWDG(Independent Watch Dog),
 * 		SPI/I2S 2-3, I2S(Inter-IC Sound) ext 2-3,
 * 		USART 2-3, UART 4-5 & 7-8,
 * 		I2C(Inter-Integrated Circuit) 1-3, I2C FMP (Fast Mode Plus) 1,
 * 		CAN (Controller Area Network ) 1-3,
 * 		PWR (Power controller) and DAC (Digital-to-Analog Converter)
 ***********************************************************************************/
#define	TIM2_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 0 ))
#define	TIM3_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 1 ))
#define	TIM4_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 2 ))
#define	TIM5_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 3 ))
#define	TIM6_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 4 ))
#define	TIM7_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 5 ))

#define	TIM12_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 6 ))
#define	TIM13_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 7 ))
#define	TIM14_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 8 ))
#define	LPTIM1_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 9 ))

#define	RTC_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 10))
#define	WWDG_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 11))

#define SPI2_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 15))

#define USART2_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 20))

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 23))
#define I2CFMP1_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 24))

#define CAN1_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 25))
#define CAN2_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 26))
#define CAN3_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 27))

#define PWR_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 28))
#define DAC_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 29))

#define UART7_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 30))
#define UART8_PCLK_EN()				(RCC->APB1ENR |= ( 1 << 31))

/************************************************************************************
 * APB2: TIM 1 8 9-11, USART1-2, UART9-10, SYSCFG, SPI 1 4 5. EXTI
*************************************************************************************/
#define TIM1_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 0 ))
#define TIM8_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 1 ))

#define USART1_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 4 ))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 5 ))
#define UART9_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 6 ))
#define UART10_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 7 ))

#define ADC1_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 8 ))

#define SDIO_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 11))

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 12))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 13))

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 14))
//#define EXTI_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 15))

#define TIM9_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 16))
#define TIM10_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 17))
#define TIM11_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 18))

#define SPI5_PCLK_EN()				(RCC->APB2ENR |= ( 1 << 20))


/*RCClock DISables for buses*/
/***********************************************************************************
 *APB1:	TIMs 2-7 & 12-14, LPTIM1, RTC(Real-Time Clock) & BKP registers,
 * 		WWDG(Window Watch Dog), IWDG(Independent Watch Dog),
 * 		SPI/I2S 2-3, I2S(Inter-IC Sound) ext 2-3,
 * 		USART 2-3, UART 4-5 & 7-8,
 * 		I2C(Inter-Integrated Circuit) 1-3, I2C FMP (Fast Mode Plus) 1,
 * 		CAN (Controller Area Network ) 1-3,
 * 		PWR (Power controller) and DAC (Digital-to-Analog Converter)
 ***********************************************************************************/
#define	TIM2_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 0 ))
#define	TIM3_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 1 ))
#define	TIM4_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 2 ))
#define	TIM5_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 3 ))
#define	TIM6_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 4 ))
#define	TIM7_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 5 ))

#define	TIM12_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 6 ))
#define	TIM13_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 7 ))
#define	TIM14_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 8 ))
#define	LPTIM1_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 9 ))

#define	RTC_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 10))
#define	WWDG_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 11))

#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 15))

#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 20))

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 23))
#define I2CFMP1_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 24))

#define CAN1_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 25))
#define CAN2_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 26))
#define CAN3_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 27))

#define PWR_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 28))
#define DAC_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 29))

#define UART7_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 30))
#define UART8_PCLK_DI()				(RCC->APB1ENR &= ~( 1 << 31))

/************************************************************************************
 * APB2: TIM 1 8 9-11, USART1-2, UART9-10, SYSCFG, SPI 1 4 5. EXTI
*************************************************************************************/
#define TIM1_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 0 ))
#define TIM8_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 1 ))

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 4 ))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 5 ))
#define UART9_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 6 ))
#define UART10_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 7 ))

#define ADC1_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 8 ))

#define SDIO_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 11))

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 12))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 13))

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 14))
//#define EXTI_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 15))

#define TIM9_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 16))
#define TIM10_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 17))
#define TIM11_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 18))

#define SPI5_PCLK_DI()				(RCC->APB2ENR &= ~( 1 << 20))


//AHB1
//Clock enable/disable for GPIOx Peripherals
#define  GPIOA_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 0 ))
#define  GPIOB_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 1 ))
#define  GPIOC_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 2 ))
#define  GPIOD_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 3 ))
#define  GPIOE_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 4 ))
#define  GPIOF_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 5 ))
#define  GPIOG_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 6 ))
#define  GPIOH_PCLK_EN()			(RCC->AHB1ENR |= ( 1 << 7 ))

#define  GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 0 ))
#define  GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 1 ))
#define  GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 2 ))
#define  GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 3 ))
#define  GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 4 ))
#define  GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 5 ))
#define  GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 6 ))
#define  GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~( 1 << 7 ))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0));\
							   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()  	do{(RCC->AHB1RSTR |= (1<<1));\
							   (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()  	do{(RCC->AHB1RSTR |= (1<<2));\
							   (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<3));\
		   	   	   	   	   	   (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<4));\
		   	   	   	   	   	   (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<5));\
		   	   	   	   	   	   (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<6));\
		   	   	   	   	   	   (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7));\
		   	   	   	   	   	   (RCC->AHB1RSTR &= ~(1<<7));}while(0)
/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<12));\
							   (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1<<14));\
							   (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1<<15));\
							   (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1<<13));\
							   (RCC->APB2RSTR &= ~(1<<13));}while(0)
#define SPI5_REG_RESET()	do{(RCC->APB2RSTR |= (1<<20));\
							   (RCC->APB2RSTR &= ~(1<<20));}while(0)


#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<4));\
							(RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1<<17));\
							(RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()	do{(RCC->APB1RSTR |= (1<<18));\
							(RCC->APB1RSTR &= ~(1<<18));}while(0)
#define USART6_REG_RESET()	do{(RCC->APB2RSTR |= (1<<5));\
							(RCC->APB2RSTR &= ~(1<<5));}while(0)
#define UART4_REG_RESET()	do{(RCC->APB1RSTR |= (1<<19));\
							(RCC->APB1RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()	do{(RCC->APB1RSTR |= (1<<20));\
							(RCC->APB1RSTR &= ~(1<<20));}while(0)
#define UART7_REG_RESET()	do{(RCC->APB1RSTR |= (1<<30));\
							(RCC->APB1RSTR &= ~(1<<30));}while(0)
#define UART8_REG_RESET()	do{(RCC->APB1RSTR |= (1<<31));\
							(RCC->APB1RSTR &= ~(1<<31));}while(0)
#define UART9_REG_RESET()	do{(RCC->APB2RSTR |= (1<<6));\
							(RCC->APB2RSTR &= ~(1<<6));}while(0)
#define UART10_REG_RESET()	do{(RCC->APB2RSTR |= (1<<7));\
							(RCC->APB2RSTR &= ~(1<<7));}while(0)

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)	( 	(x == GPIOA)? 0 : \
										(x == GPIOB)? 1 : \
										(x == GPIOC)? 2 : \
										(x == GPIOD)? 3 : \
										(x == GPIOE)? 4 : \
										(x == GPIOF)? 5 : \
										(x == GPIOG)? 6 : \
										(x == GPIOH)? 7 : 0	)
/*
 * IRQ(Interrupt Request) Vector Table position Numbers of STM32F413xx MCU
 */
enum vector_table_positions{
	IRQ_VPOS_WWDG = 0,					//Window watchdog
	IRQ_VPOS_PVD,						//PVD through EXTI
	IRQ_VPOS_TAMP_STAMP,				//Tamper and TimeStamp
	IRQ_VPOS_RTC_WKUP,					//RTC Wakeup
	IRQ_VPOS_FLASH,						//Flash global interrupt
	IRQ_VPOS_RCC,						//Reset & Clock Control global interrupt
	IRQ_VPOS_EXTI0,
	IRQ_VPOS_EXTI1,
	IRQ_VPOS_EXTI2,
	IRQ_VPOS_EXTI3,
	IRQ_VPOS_EXTI4,
	IRQ_VPOS_DMA1_STREAM0,
	IRQ_VPOS_DMA1_STREAM1,
	IRQ_VPOS_DMA1_STREAM2,
	IRQ_VPOS_DMA1_STREAM3,
	IRQ_VPOS_DMA1_STREAM4,
	IRQ_VPOS_DMA1_STREAM5,
	IRQ_VPOS_DMA1_STREAM6,
	IRQ_VPOS_ADC,
	IRQ_VPOS_CAN1_TX,
	IRQ_VPOS_CAN1_RX0,
	IRQ_VPOS_CAN1_RX1,
	IRQ_VPOS_CAN1_SCE,
	IRQ_VPOS_EXTI9_5,					//EXTI line[9:5] interrupts
	IRQ_VPOS_TIM1_BRK_TIM9,				//TIM1 Break interrupt and TIM9 interrupt
	IRQ_VPOS_TIM1_UP_TIM10,				//TIM1 update interrupt and TIM10 interrupt
	IRQ_VPOS_TIM_TRG_COM_TIM11,			/*TIM1 trigger&conmmutation interrupt
										and TIM11 interrupt*/
	IRQ_VPOS_TIM1_CC,					//TIM1 Capture-Compare interrupt
	IRQ_VPOS_TIM2,
	IRQ_VPOS_TIM3,
	IRQ_VPOS_TIM4,
	IRQ_VPOS_I2C1_EVT,					//I2C1 event interrupt
	IRQ_VPOS_I2C1_ERR,					//I2C1 error interrupt
	IRQ_VPOS_I2C2_EVT,					//I2C2 event interrupt
	IRQ_VPOS_I2C2_ERR,					//I2C2 error interrupt
	IRQ_VPOS_SPI1,
	IRQ_VPOS_SPI2,
	IRQ_VPOS_USART1,
	IRQ_VPOS_USART2,
	IRQ_VPOS_USART3,
	IRQ_VPOS_EXTI15_10,					//EXTI line[15:10] interrupts
	IRQ_VPOS_EXTI17,
	IRQ_VPOS_EXTI18,
	IRQ_VPOS_TIM8_BRK_TIM12,			//TIM8 Break interrupt and TIM12 interrupt
	IRQ_VPOS_TIM8_UP_TIM13,				//TIM8 update interrupt and TIM13 interrupt
	IRQ_VPOS_TIM8_TRG_COM_TIM14,		/*TIM8 trigger&conmmutation interrupt
										and TIM14 interrupt*/
	IRQ_VPOS_TIM8_CC,					//TIM8 Capture-Compare interrupt
	IRQ_VPOS_DMA1_STREAM7,
	IRQ_VPOS_FSMC,
	IRQ_VPOS_SDIO,
	IRQ_VPOS_TIM5,
	IRQ_VPOS_SPI3,
	IRQ_VPOS_UART4,
	IRQ_VPOS_UART5,
	IRQ_VPOS_TIM6_GLB_IT,
	IRQ_VPOS_TIM6,
	IRQ_VPOS_DMA2_STREAM0,
	IRQ_VPOS_DMA2_STREAM1,
	IRQ_VPOS_DMA2_STREAM2,
	IRQ_VPOS_DMA2_STREAM3,
	IRQ_VPOS_DMA2_STREAM4,
	IRQ_VPOS_DFSDM1_FLT0,
	IRQ_VPOS_DFSDM1_FLT1,
	IRQ_VPOS_CAN2_TX,
	IRQ_VPOS_CAN2_RX0,
	IRQ_VPOS_CAN2_RX1,
	IRQ_VPOS_CAN2_SCE,
	IRQ_VPOS_OTG_FS,
	IRQ_VPOS_DMA2_STREAM5,
	IRQ_VPOS_DMA2_STREAM6,
	IRQ_VPOS_DMA2_STREAM7,
	IRQ_VPOS_USART6,
	IRQ_VPOS_I2C3_EVT,
	IRQ_VPOS_I2C3_ERR,
	IRQ_VPOS_CAN3_TX,
	IRQ_VPOS_CAN3_RX0,
	IRQ_VPOS_CAN3_RX1,
	IRQ_VPOS_CAN3_SCE,
	IRQ_VPOS_CRYPTO=79,
	IRQ_VPOS_RNG,
	IRQ_VPOS_FPU,
	IRQ_VPOS_UART7,
	IRQ_VPOS_UART8,
	IRQ_VPOS_SPI4,
	IRQ_VPOS_SPI5,
	IRQ_VPOS_SAI1=87,
	IRQ_VPOS_UART9,
	IRQ_VPOS_UART10,
	IRQ_VPOS_QUADSPI=92,
	IRQ_VPOS_I2CFMP1_EVT=95,
	IRQ_VPOS_I2CFMP1_ERR,
	IRQ_VPOS_EXTI23,
	IRQ_VPOS_DFSDM2_FLT0,
	IRQ_VPOS_DFSDM2_FLT1,
	IRQ_VPOS_DFSDM2_FLT2,
	IRQ_VPOS_DFSDM2_FLT3,
};
#define IRQ_VPOS_RTC_ALARM			IRQ_VPOS_EXTI17
#define IRQ_VPOS_OTG_FS_WKUP		IRQ_VPOS_EXTI18
#define IRQ_VPOS_DAC1				IRQ_VPOS_TIM6_GLB_IT
#define IRQ_VPOS_DAC2				IRQ_VPOS_TIM6_GLB_IT
#define IRQ_VPOS_LPTIM1				IRQ_VPOS_EXTI23

#include "stm32f413xx_gpio_driver.h"
#include "stm32f413xx_spi_driver.h"
#include "stm32f413xx_rcc_driver.h"
#include "stm32f4xx_usart_driver.h"

#endif /* INC_STM32F413XX_H_ */
