/**
 ******************************************************************************
 * @file      stm32f4_startup.c
 * @author    alexandergmzx
 * @brief     STM32F4 device vector table for GCC toolchain. Now written in C.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the initial PC == Reset_Handler,
 *                - Set the vector table entries with the exceptions ISR address
 *                - Branches to main in the C library (which eventually
 *                  calls main()).
 ******************************************************************************
 */
#include <stdint.h>

#define SRAM_START	(0x20000000UL)
#define SRAM_SIZE	(320 * 1024U) // 320 kB
#define SRAM_END	(SRAM_START + SRAM_SIZE)

#define STACK_START   SRAM_END

extern uint32_t _etext;
/* start address for the .data section. defined in linker script */
extern uint32_t _sdata;
/* end address for the .data section. defined in linker script */
extern uint32_t _edata;

/* start address for the .bss section. defined in linker script */
extern uint32_t _sbss;
/* end address for the .bss section. defined in linker script */
extern uint32_t _ebss;

/* start address for the initialization values of the .data section.
defined in linker script */
extern uint32_t _sidata;

/* Highest address of the user mode stack, equal to STACK_START */
extern uint32_t _estack;

/* Entry point for the application. */
extern int main(void);
/* Initialize the libc std library */
extern void __libc_init_array(void);

/* External declaration for system initialization function                  */
void SystemInit(void); __attribute__((weak));

__attribute__((weak,interrupt,section(".text:Reset_Handler")))
void Reset_Handler(void){
	/* Set stack pointer */
	/*__asm("	ldr   r0, =_estack\n"
  		  "	mov   sp, r0");*/

	//copy .data section to SRAM:
	// Get the address of the end and the start of data section
	
	uint32_t *pSrc = (uint32_t*)&_sidata; //flash
	uint32_t *pDst = (uint32_t*)&_sdata; //sram
	
	uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
	
	/* Copy the data segment initializers from flash to SRAM */
	for(uint32_t i = 0; i < size; i++)
		*pDst++ = *pSrc++;
	
	/* //This does the same: copy .data section to SRAM
	uint32_t *pui32Src, *pui32Dest;
	pui32Src = &_sidata;
    for(pui32Dest = &_sdata; pui32Dest < &_edata; )
    {
        *pui32Dest++ = *pui32Src++;
    }
	*/
	
	//Init. the .bss section to zero in SRAM
	size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	pDst = (uint32_t*)&_sbss;
	for(uint32_t i =0 ; i < size ; i++)
		*pDst++ = 0;
	
	/* Call system initialization routine */
	//SystemInit();
	/* Call static constructors */
	__libc_init_array();
	/* Call the application's entry point. */ 
	main();

}

__attribute__((interrupt,section(".text:Default_Handler")))
void Default_Handler(void){
	while(1);
}

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
/* Function prototypes of system exception and IRQ handlers*/
void Default_Handler(void);
void Reset_Handler(void);
void NMI_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void HardFault_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void MemManage_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void BusFault_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void UsageFault_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void SVC_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void DebugMon_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void PendSV_Handler(void)	__attribute__((weak,alias("Default_Handler")));
void SysTick_Handler(void)	__attribute__((weak,alias("Default_Handler")));                               			/* Reserved                                                           */
void PVD_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                  			/* PVD through EXTI line detection interrupt                          */
void TAMP_STAMP_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));           			/* Tamper and TimeStamp interrupts through the EXTI line              */
void RTC_WKUP_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* RTC Wakeup interrupt through the EXTI line                         */
void FLASH_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* FLASH global interrupt                                             */
void RCC_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                  			/* RCC global interrupt                                               */
void EXTI0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* EXTI Line0 interrupt                                               */
void EXTI1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* EXTI Line1 interrupt                                               */
void EXTI2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* EXTI Line2 interrupt                                               */
void EXTI3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* EXTI Line3 interrupt                                               */
void EXTI4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* EXTI Line4 interrupt                                               */
void DMA1_Stream0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream0 global interrupt                                      */
void DMA1_Stream1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream1 global interrupt                                      */
void DMA1_Stream2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream2 global interrupt                                      */
void DMA1_Stream3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream3 global interrupt                                      */
void DMA1_Stream4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream4 global interrupt                                      */
void DMA1_Stream5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream5 global interrupt                                      */
void DMA1_Stream6_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 Stream6 global interrupt                                      */
void ADC_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                  			/* ADC1 global interrupt                                              */
void CAN1_TX_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* CAN1 TX interrupts                                                 */
void CAN1_RX0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* CAN1 RX0 interrupts                                                */
void CAN1_RX1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* CAN1 RX1 interrupts                                                */
void CAN1_SCE_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* CAN1 SCE interrupt                                                 */
void EXTI9_5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* EXTI Line[9:5] interrupts                                          */
void TIM1_BRK_TIM9_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));        			/* TIM1 Break interrupt and TIM9 global interrupt                     */
void TIM1_UP_TIM10_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));        			/* TIM1 Update interrupt and TIM10 global interrupt                   */
void TIM1_TRG_COM_TIM11_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));   			/* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt */
void TIM1_CC_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* TIM1 Capture Compare interrupt                                     */
void TIM2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* TIM2 global interrupt                                              */
void TIM3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* TIM3 global interrupt                                              */
void TIM4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* TIM4 global interrupt                                              */
void I2C1_EVT_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* I2C1 event interrupt                                               */
void I2C1_ERR_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* I2C1 error interrupt                                               */
void I2C2_EVT_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* I2C2 event interrupt                                               */
void I2C2_ERR_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* I2C2 error interrupt                                               */
void SPI1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SPI1 global interrupt                                              */
void SPI2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SPI2 global interrupt                                              */
void USART1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART1 global interrupt                                            */
void USART2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART2 global interrupt                                            */
void USART3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART3 global interrupt                                            */
void EXTI15_10_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));            			/* EXTI Line[15:10] interrupts                                        */
void EXTI17_RTC_Alarm_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));     			/* RTC Alarms (A and B) through EXTI line interrupt                   */                               			/* Reserved                                                           */
void TIM8_BRK_TIM12_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));       			/* Timer 12 global interrupt                                          */
void TIM8_UP_TIM13_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));        			/* Timer 13 global interrupt                                          */
void TIM8_TRG_COM_TIM14_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));   			/* Timer 14 global interrupt                                          */
void TIM8_CC_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* TIM8 Cap/Com interrupt                                             */
void DMA1_Stream7_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA1 global interrupt Channel 7                                    */
void FSMC_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* FSMC global interrupt                                              */
void SDIO_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SDIO global interrupt                                              */
void TIM5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* TIM5 global interrupt                                              */
void SPI3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SPI3 global interrupt                                              */
void USART4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* UART 4 global interrupt                                            */
void UART5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* UART 5global interrupt                                             */
void TIM6_GLB_IT_DAC1_DAC2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));			/* TIM6 global and DAC12 underrun interrupts                          */
void TIM7_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* TIM7 global interrupt                                              */
void DMA2_Stream0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream0 global interrupt                                      */
void DMA2_Stream1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream1 global interrupt                                      */
void DMA2_Stream2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream2 global interrupt                                      */
void DMA2_Stream3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream3 global interrupt                                      */
void DMA2_Stream4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream4 global interrupt                                      */
void DFSDM1_FLT0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));          			/* SD filter0 global interrupt                                        */
void DFSDM1_FLT1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));          			/* SD filter1 global interrupt                                        */
void CAN2_TX_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* CAN2 TX interrupt                                                  */
void CAN2_RX0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* BXCAN2 RX0 interrupt                                               */
void CAN2_RX1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* BXCAN2 RX1 interrupt                                               */
void CAN2_SCE_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* CAN2 SCE interrupt                                                 */                               			/* Reserved                                                           */
void DMA2_Stream5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream5 global interrupt                                      */
void DMA2_Stream6_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream6 global interrupt                                      */
void DMA2_Stream7_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* DMA2 Stream7 global interrupt                                      */
void USART6_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART6 global interrupt                                            */
void I2C3_EV_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* I2C3 event interrupt                                               */
void I2C3_ER_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* I2C3 error interrupt                                               */
void CAN3_TX_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* CAN 3 TX interrupt                                                 */
void CAN3_RX0_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* BxCAN 3 RX0 interrupt                                              */
void CAN3_RX1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* BxCAN 3 RX1 interrupt                                              */
void CAN3_SCE_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));             			/* CAN 3 SCE interrupt                                                */                               			/* Reserved                                                           */
void CRYPTO_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* AES global interrupt                                               */
void RNG_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                  			/* Rng global interrupt                                               */
void FPU_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                  			/* Floating point interrupt                                           */
void USART7_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART7 global interrupt                                            */
void USART8_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* USART8 global interrupt                                            */
void SPI4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SPI4 global interrupt                                              */
void SPI5_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SPI5 global interrupt                                              */                               			/* Reserved                                                           */
void SAI1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                 			/* SAI1 global interrupt                                              */
void UART9_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));                			/* UART9 global interrupt                                             */
void UART10_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));               			/* UART10 global interrupt                                            */                               			/* Reserved                                                           */                               			/* Reserved                                                           */
void QuadSPI_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));              			/* Quad-SPI global interrupt                                          */                               			/* Reserved                                                           */                               			/* Reserved                                                           */
void I2CFMP1event_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* I2CFMP1 event interrupt                                            */
void I2CFMP1error_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));         			/* I2CFMP1 error interrupt                                            */
void lptim1_OR_it_eit_23_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));  			/* LP Timer global interrupt or EXT1 interrupt line 23                */
void DFSDM2_FILTER1_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));       			/* DFSDM2 SD filter 1 global interrupt                                */
void DFSDM2_FILTER2_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));       			/* DFSDM2 SD filter 2 global interrupt                                */
void DFSDM2_FILTER3_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));       			/* DFSDM2 SD filter 3 global interrupt                                */
void DFSDM2_FILTER4_IRQHandler(void)	__attribute__((weak,alias("Default_Handler")));       			/* DFSDM2 SD filter 4 global interrupt                                */

void (* const interruptVectors[])(void) __attribute__ ((section (".isr_vector"))) = 
{
	STACK_START,
	(uint32_t)&Reset_Handler,
	(uint32_t)&NMI_Handler,
	(uint32_t)&MemManage_Handler,
	(uint32_t)&BusFault_Handler,
	(uint32_t)&UsageFault_Handler,
	0,
	0,
	0,
	0,
	(uint32_t)&SVC_Handler,
	(uint32_t)&DebugMon_Handler,
	0,
	(uint32_t)&PendSV_Handler,
	(uint32_t)&SysTick_Handler,
	0                               			/* Reserved                                                           */,
	(uint32_t)&PVD_IRQHandler                  			/* PVD through EXTI line detection interrupt                          */,
	(uint32_t)&TAMP_STAMP_IRQHandler           			/* Tamper and TimeStamp interrupts through the EXTI line              */,
	(uint32_t)&RTC_WKUP_IRQHandler             			/* RTC Wakeup interrupt through the EXTI line                         */,
	(uint32_t)&FLASH_IRQHandler                			/* FLASH global interrupt                                             */,
	(uint32_t)&RCC_IRQHandler                  			/* RCC global interrupt                                               */,
	(uint32_t)&EXTI0_IRQHandler                			/* EXTI Line0 interrupt                                               */,
	(uint32_t)&EXTI1_IRQHandler                			/* EXTI Line1 interrupt                                               */,
	(uint32_t)&EXTI2_IRQHandler                			/* EXTI Line2 interrupt                                               */,
	(uint32_t)&EXTI3_IRQHandler                			/* EXTI Line3 interrupt                                               */,
	(uint32_t)&EXTI4_IRQHandler                			/* EXTI Line4 interrupt                                               */,
	(uint32_t)&DMA1_Stream0_IRQHandler         			/* DMA1 Stream0 global interrupt                                      */,
	(uint32_t)&DMA1_Stream1_IRQHandler         			/* DMA1 Stream1 global interrupt                                      */,
	(uint32_t)&DMA1_Stream2_IRQHandler         			/* DMA1 Stream2 global interrupt                                      */,
	(uint32_t)&DMA1_Stream3_IRQHandler         			/* DMA1 Stream3 global interrupt                                      */,
	(uint32_t)&DMA1_Stream4_IRQHandler         			/* DMA1 Stream4 global interrupt                                      */,
	(uint32_t)&DMA1_Stream5_IRQHandler         			/* DMA1 Stream5 global interrupt                                      */,
	(uint32_t)&DMA1_Stream6_IRQHandler         			/* DMA1 Stream6 global interrupt                                      */,
	(uint32_t)&ADC_IRQHandler                  			/* ADC1 global interrupt                                              */,
	(uint32_t)&CAN1_TX_IRQHandler              			/* CAN1 TX interrupts                                                 */,
	(uint32_t)&CAN1_RX0_IRQHandler             			/* CAN1 RX0 interrupts                                                */,
	(uint32_t)&CAN1_RX1_IRQHandler             			/* CAN1 RX1 interrupts                                                */,
	(uint32_t)&CAN1_SCE_IRQHandler             			/* CAN1 SCE interrupt                                                 */,
	(uint32_t)&EXTI9_5_IRQHandler              			/* EXTI Line[9:5] interrupts                                          */,
	(uint32_t)&TIM1_BRK_TIM9_IRQHandler        			/* TIM1 Break interrupt and TIM9 global interrupt                     */,
	(uint32_t)&TIM1_UP_TIM10_IRQHandler        			/* TIM1 Update interrupt and TIM10 global interrupt                   */,
	(uint32_t)&TIM1_TRG_COM_TIM11_IRQHandler   			/* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt */,
	(uint32_t)&TIM1_CC_IRQHandler              			/* TIM1 Capture Compare interrupt                                     */,
	(uint32_t)&TIM2_IRQHandler                 			/* TIM2 global interrupt                                              */,
	(uint32_t)&TIM3_IRQHandler                 			/* TIM3 global interrupt                                              */,
	(uint32_t)&TIM4_IRQHandler                 			/* TIM4 global interrupt                                              */,
	(uint32_t)&I2C1_EVT_IRQHandler             			/* I2C1 event interrupt                                               */,
	(uint32_t)&I2C1_ERR_IRQHandler             			/* I2C1 error interrupt                                               */,
	(uint32_t)&I2C2_EVT_IRQHandler             			/* I2C2 event interrupt                                               */,
	(uint32_t)&I2C2_ERR_IRQHandler             			/* I2C2 error interrupt                                               */,
	(uint32_t)&SPI1_IRQHandler                 			/* SPI1 global interrupt                                              */,
	(uint32_t)&SPI2_IRQHandler                 			/* SPI2 global interrupt                                              */,
	(uint32_t)&USART1_IRQHandler               			/* USART1 global interrupt                                            */,
	(uint32_t)&USART2_IRQHandler               			/* USART2 global interrupt                                            */,
	(uint32_t)&USART3_IRQHandler               			/* USART3 global interrupt                                            */,
	(uint32_t)&EXTI15_10_IRQHandler            			/* EXTI Line[15:10] interrupts                                        */,
	(uint32_t)&EXTI17_RTC_Alarm_IRQHandler     			/* RTC Alarms (A and B) through EXTI line interrupt                   */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&TIM8_BRK_TIM12_IRQHandler       			/* Timer 12 global interrupt                                          */,
	(uint32_t)&TIM8_UP_TIM13_IRQHandler        			/* Timer 13 global interrupt                                          */,
	(uint32_t)&TIM8_TRG_COM_TIM14_IRQHandler   			/* Timer 14 global interrupt                                          */,
	(uint32_t)&TIM8_CC_IRQHandler              			/* TIM8 Cap/Com interrupt                                             */,
	(uint32_t)&DMA1_Stream7_IRQHandler         			/* DMA1 global interrupt Channel 7                                    */,
	(uint32_t)&FSMC_IRQHandler                 			/* FSMC global interrupt                                              */,
	(uint32_t)&SDIO_IRQHandler                 			/* SDIO global interrupt                                              */,
	(uint32_t)&TIM5_IRQHandler                 			/* TIM5 global interrupt                                              */,
	(uint32_t)&SPI3_IRQHandler                 			/* SPI3 global interrupt                                              */,
	(uint32_t)&USART4_IRQHandler               			/* UART 4 global interrupt                                            */,
	(uint32_t)&UART5_IRQHandler                			/* UART 5global interrupt                                             */,
	(uint32_t)&TIM6_GLB_IT_DAC1_DAC2_IRQHandler			/* TIM6 global and DAC12 underrun interrupts                          */,
	(uint32_t)&TIM7_IRQHandler                 			/* TIM7 global interrupt                                              */,
	(uint32_t)&DMA2_Stream0_IRQHandler         			/* DMA2 Stream0 global interrupt                                      */,
	(uint32_t)&DMA2_Stream1_IRQHandler         			/* DMA2 Stream1 global interrupt                                      */,
	(uint32_t)&DMA2_Stream2_IRQHandler         			/* DMA2 Stream2 global interrupt                                      */,
	(uint32_t)&DMA2_Stream3_IRQHandler         			/* DMA2 Stream3 global interrupt                                      */,
	(uint32_t)&DMA2_Stream4_IRQHandler         			/* DMA2 Stream4 global interrupt                                      */,
	(uint32_t)&DFSDM1_FLT0_IRQHandler          			/* SD filter0 global interrupt                                        */,
	(uint32_t)&DFSDM1_FLT1_IRQHandler          			/* SD filter1 global interrupt                                        */,
	(uint32_t)&CAN2_TX_IRQHandler              			/* CAN2 TX interrupt                                                  */,
	(uint32_t)&CAN2_RX0_IRQHandler             			/* BXCAN2 RX0 interrupt                                               */,
	(uint32_t)&CAN2_RX1_IRQHandler             			/* BXCAN2 RX1 interrupt                                               */,
	(uint32_t)&CAN2_SCE_IRQHandler             			/* CAN2 SCE interrupt                                                 */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&DMA2_Stream5_IRQHandler         			/* DMA2 Stream5 global interrupt                                      */,
	(uint32_t)&DMA2_Stream6_IRQHandler         			/* DMA2 Stream6 global interrupt                                      */,
	(uint32_t)&DMA2_Stream7_IRQHandler         			/* DMA2 Stream7 global interrupt                                      */,
	(uint32_t)&USART6_IRQHandler               			/* USART6 global interrupt                                            */,
	(uint32_t)&I2C3_EV_IRQHandler              			/* I2C3 event interrupt                                               */,
	(uint32_t)&I2C3_ER_IRQHandler              			/* I2C3 error interrupt                                               */,
	(uint32_t)&CAN3_TX_IRQHandler              			/* CAN 3 TX interrupt                                                 */,
	(uint32_t)&CAN3_RX0_IRQHandler             			/* BxCAN 3 RX0 interrupt                                              */,
	(uint32_t)&CAN3_RX1_IRQHandler             			/* BxCAN 3 RX1 interrupt                                              */,
	(uint32_t)&CAN3_SCE_IRQHandler             			/* CAN 3 SCE interrupt                                                */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&CRYPTO_IRQHandler               			/* AES global interrupt                                               */,
	(uint32_t)&RNG_IRQHandler                  			/* Rng global interrupt                                               */,
	(uint32_t)&FPU_IRQHandler                  			/* Floating point interrupt                                           */,
	(uint32_t)&USART7_IRQHandler               			/* USART7 global interrupt                                            */,
	(uint32_t)&USART8_IRQHandler               			/* USART8 global interrupt                                            */,
	(uint32_t)&SPI4_IRQHandler                 			/* SPI4 global interrupt                                              */,
	(uint32_t)&SPI5_IRQHandler                 			/* SPI5 global interrupt                                              */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&SAI1_IRQHandler                 			/* SAI1 global interrupt                                              */,
	(uint32_t)&UART9_IRQHandler                			/* UART9 global interrupt                                             */,
	(uint32_t)&UART10_IRQHandler               			/* UART10 global interrupt                                            */,
	0                               			/* Reserved                                                           */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&QuadSPI_IRQHandler              			/* Quad-SPI global interrupt                                          */,
	0                               			/* Reserved                                                           */,
	0                               			/* Reserved                                                           */,
	(uint32_t)&I2CFMP1event_IRQHandler         			/* I2CFMP1 event interrupt                                            */,
	(uint32_t)&I2CFMP1error_IRQHandler         			/* I2CFMP1 error interrupt                                            */,
	(uint32_t)&lptim1_OR_it_eit_23_IRQHandler  			/* LP Timer global interrupt or EXT1 interrupt line 23                */,
	(uint32_t)&DFSDM2_FILTER1_IRQHandler       			/* DFSDM2 SD filter 1 global interrupt                                */,
	(uint32_t)&DFSDM2_FILTER2_IRQHandler       			/* DFSDM2 SD filter 2 global interrupt                                */,
	(uint32_t)&DFSDM2_FILTER3_IRQHandler       			/* DFSDM2 SD filter 3 global interrupt                                */,
	(uint32_t)&DFSDM2_FILTER4_IRQHandler       			/* DFSDM2 SD filter 4 global interrupt                                */,
};

