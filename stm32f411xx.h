/*
 * stm32f411xx.h
 *
 *  Created on: Jul 30, 2024
 *      Author: tonys
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include <stdint.h>


#define __IO                    volatile

#define ENABLE                  1
#define DISABLE                 0

#define SET                     ENABLE
#define RESET                   DISABLE
#define CLEAR                   DISABEL

#define LOW                     0
#define HIGH                    1

#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

#define FLAG_SET                SET
#define FLAG_RESET              RESET



/***********************************************************
 *                   Base Address Defines
 ***********************************************************/
/**
 * @brief Base addresses of FLASH and SRAM
 */
#define FLASH_BASEADDR          0x08000000U /*!<base address of main flash */
#define ROM_BASEADDR            0x1FFF0000U
#define SRAM1_BASEADDR          0x20000000U
#define SRAM                    SRAM_BASEADDR

/**
 * @brief Base addresses of bus domains
 */
#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U

/**
 * @brief Base addresses of peripherals attached to AHB1 bus
 * @todo TODO: Complete for all other peripherals and busses
 */
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)


/**
 * @brief Base addresses of peripherals attached to APB1 bus
 * TODO: Complete for all other peripherals and busses
 */
#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400)


/**
 * @brief Base addresses of peripherals attached to APB2 bus
 * TODO: Complete for all other peripherals and busses
 */
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR           (APB2PERIPH_BASEADDR + 0x5000)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)



/***********************************************************
 * @brief ARM Cortex-Mx Processor NVIC ISERx Register Addresses
 ***********************************************************/
#define NVIC_ISER0              ((__IO uint32_t *)0xE000E100)
#define NVIC_ISER1              ((__IO uint32_t *)0xE000E104)
#define NVIC_ISER2              ((__IO uint32_t *)0xE000E108)
#define NVIC_ISER3              ((__IO uint32_t *)0xE000E10C)



/***********************************************************
 * @brief ARM Cortex-Mx Processor NVIC ICERx Register Addresses
 ***********************************************************/
#define NVIC_ICER0              ((__IO uint32_t *)0xE000E180)
#define NVIC_ICER1              ((__IO uint32_t *)0xE000E184)
#define NVIC_ICER2              ((__IO uint32_t *)0xE000E188)
#define NVIC_ICER3              ((__IO uint32_t *)0xE000E18C)



/***********************************************************
 * @brief ARM Cortex-Mx Processor NVIC IPRx Register Addresses
 ***********************************************************/
#define NVIC_PRI_BASEADDR       ((__IO uint32_t *)0xE000E400)

/**
 * @brief Number of bits used for Priority Levels in STM32
 * This is different from TI Tiva which uses 8 bits of
 * priority levels
 */
#define NO_BITS_IMPLEMENTED     4



/***********************************************************
 * @brief            RCC
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for RCC
 */
typedef struct
{
	__IO uint32_t CR;           /*!< GPIO Port Mode Register, Addr offset: 0x00*/
	__IO uint32_t PLLCFGR;      /*!< GPIO Port Mode Register, Addr offset: 0x04*/
	__IO uint32_t CFGR;         /*!< GPIO Port Mode Register, Addr offset: 0x08*/
	__IO uint32_t CIR;          /*!< GPIO Port Mode Register, Addr offset: 0x0C*/
	__IO uint32_t AHB1RSTR;     /*!< GPIO Port Mode Register, Addr offset: 0x10*/
	__IO uint32_t AHB2RSTR;     /*!< GPIO Port Mode Register, Addr offset: 0x14*/
	uint32_t      RESERVED0[2]; /*!< GPIO Port Mode Register, Addr offset: 0x18 - 0x1C*/
	__IO uint32_t APB1RSTR;     /*!< GPIO Port Mode Register, Addr offset: 0x20*/
	__IO uint32_t APB2RSTR;     /*!< GPIO Port Mode Register, Addr offset: 0x24*/
	uint32_t      RESERVED1[2]; /*!< GPIO Port Mode Register, Addr offset: 0x28 - 0x2C*/
	__IO uint32_t AHB1ENR;      /*!< GPIO Port Mode Register, Addr offset: 0x30*/
	__IO uint32_t AHB2ENR;      /*!< GPIO Port Mode Register, Addr offset: 0x34*/
	uint32_t      RESERVED2[2]; /*!< GPIO Port Mode Register, Addr offset: 0x38 - 0x3C*/

	__IO uint32_t APB1ENR;      /*!< GPIO Port Mode Register, Addr offset: 0x40*/
	__IO uint32_t APB2ENR;      /*!< GPIO Port Mode Register, Addr offset: 0x44*/
	uint32_t      RESERVED3[2]; /*!< GPIO Port Mode Register, Addr offset: 0x48 - 0x4C*/
	__IO uint32_t AHB1LPENR;    /*!< GPIO Port Mode Register, Addr offset: 0x50*/
	__IO uint32_t AHB2LPENR;    /*!< GPIO Port Mode Register, Addr offset: 0x54*/
	uint32_t      RESERVED4[2]; /*!< GPIO Port Mode Register, Addr offset: 0x58 - 0x5C*/
	__IO uint32_t APB1LPENR;    /*!< GPIO Port Mode Register, Addr offset: 0x60*/
	__IO uint32_t APB2LPENR;    /*!< GPIO Port Mode Register, Addr offset: 0x64*/
	uint32_t      RESERVED5[2]; /*!< GPIO Port Mode Register, Addr offset: 0x68 - 0x6C*/

	__IO uint32_t BDCR;         /*!< GPIO Port Mode Register, Addr offset: 0x70*/
	__IO uint32_t CSR;          /*!< GPIO Port Mode Register, Addr offset: 0x74*/
	uint32_t      RESERVED6[2]; /*!< GPIO Port Mode Register, Addr offset: 0x78 - 0x7C*/
	__IO uint32_t SSCGR;        /*!< GPIO Port Mode Register, Addr offset: 0x80*/
	__IO uint32_t PLLI2SCFGR;   /*!< GPIO Port Mode Register, Addr offset: 0x84*/
} RCC_RegDef_t;


/**
 * @brief RCC Definition (Peripheral base addresses type casted to xxxRegDef_t)
 */
#define RCC                     ((RCC_RegDef_t *)RCC_BASEADDR)



/***********************************************************
 *                   GPIO
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for GPIO
 */
typedef struct
{
	__IO uint32_t MODER;        /*!< GPIO Port Mode Register, Addr offset: 0x00*/
	__IO uint32_t OTYPER;       /*!< short description, Addr offset: 0x04*/
	__IO uint32_t OSPEEDR;      /*!< short description, Addr offset: 0x08*/
	__IO uint32_t PUPDR;        /*!< short description, Addr offset: 0x0C*/
	__IO uint32_t IDR;          /*!< short description, Addr offset: 0x10*/
	__IO uint32_t ODR;          /*!< short description, Addr offset: 0x14*/
	__IO uint32_t BSRR;         /*!< short description, Addr offset: 0x18*/
	__IO uint32_t LCKR;         /*!< short description, Addr offset: 0x1C*/
	__IO uint32_t AFR[2];       /*!< short description, Addr offset: 0x20 - 0x24*/

} GPIO_RegDef_t;


/**
 * @brief GPIOx Definitions (Peripheral base addresses type casted to xxxRegDef_t)
 */
#define GPIOA                   ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t *)GPIOH_BASEADDR)


/**
 * @brief GPIOx peripheral reset
 */
#define GPIOA_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 0)); (RCC->AHB1ENR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 1)); (RCC->AHB1ENR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 2)); (RCC->AHB1ENR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 3)); (RCC->AHB1ENR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 4)); (RCC->AHB1ENR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB1ENR |= (1 << 7)); (RCC->AHB1ENR &= ~(1 << 7)); }while(0)



/***********************************************************
 *                   EXTI
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for EXTI
 */
typedef struct
{
	__IO uint32_t IMR;          /*!< EXTI Register,     Addr offset: 0x00*/
	__IO uint32_t EMR;          /*!< short description, Addr offset: 0x04*/
	__IO uint32_t RTSR;         /*!< short description, Addr offset: 0x08*/
	__IO uint32_t FTSR;         /*!< short description, Addr offset: 0x0C*/
	__IO uint32_t SWIER;        /*!< short description, Addr offset: 0x10*/
	__IO uint32_t PR;           /*!< short description, Addr offset: 0x14*/
} EXTI_RegDef_t;


/**
 * @brief EXTI Definition (Peripheral base addresses type casted to xxxRegDef_t)
 */
#define EXTI                    ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define EXTI_PR13               (1U << 13)



/***********************************************************
 *                   NVIC
 ***********************************************************/
/**
 * @brief NVIC IRQ numbers
 */
typedef enum
{
	/******  Cortex-M4 Specific Exceptions Numbers ************************************************************/
	NonMaskableInt_IRQn       = -14,    /*!< 2 Non Maskable Interrupt                                       > */
	MemoryManagement_IRQn     = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                          */
        BusFault_IRQn             = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                  */
	UsageFault_IRQn           = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                */
	SVCall_IRQn               = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                   */
	DebugMonitor_IRQn         = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                             */
	PendSV_IRQn               = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                   */
	SysTick_IRQn              = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                               */
	/******  STM32 Specific Interrupt Numbers *****************************************************************/
	WWDG_IRQn                  = 0,    /*!< Window WatchDog Interrupt                                         */
	PVD_IRQn                   = 1,    /*!< PVD through EXTI Line detection Interrupt                         */
	TAMP_STAMP_IRQn            = 2,    /*!< Tamper and TimeStamp interrupts through the EXTI line             */
	RTC_WKUP_IRQn              = 3,    /*!< RTC Wake up interrupt through the EXTI line                       */
	FLASH_IRQn                 = 4,    /*!< FLASH global Interrupt                                            */
	RCC_IRQn                   = 5,    /*!< RCC global Interrupt                                              */
        EXTI0_IRQn                 = 6,    /*!< EXTI Line0 Interrupt                                              */
	EXTI1_IRQn                 = 7,    /*!< EXTI Line1 Interrupt                                              */
	EXTI2_IRQn                 = 8,    /*!< EXTI Line2 Interrupt                                              */
	EXTI3_IRQn                 = 9,    /*!< EXTI Line3 Interrupt                                              */
	EXTI4_IRQn                 = 10,   /*!< EXTI Line4 Interrupt                                              */
	DMA1_Stream0_IRQn          = 11,   /*!< DMA1 Stream 0 global Interrupt                                    */
	DMA1_Stream1_IRQn          = 12,   /*!< DMA1 Stream 1 global Interrupt                                    */
	DMA1_Stream2_IRQn          = 13,   /*!< DMA1 Stream 2 global Interrupt                                    */
	DMA1_Stream3_IRQn          = 14,   /*!< DMA1 Stream 3 global Interrupt                                    */
	DMA1_Stream4_IRQn          = 15,   /*!< DMA1 Stream 4 global Interrupt                                    */
	DMA1_Stream5_IRQn          = 16,   /*!< DMA1 Stream 5 global Interrupt                                    */
	DMA1_Stream6_IRQn          = 17,   /*!< DMA1 Stream 6 global Interrupt                                    */
	ADC_IRQn                   = 18,   /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
	EXTI9_5_IRQn               = 23,   /*!< External Line[9:5] Interrupts                                     */
	TIM1_BRK_TIM9_IRQn         = 24,   /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
	TIM1_UP_TIM10_IRQn         = 25,   /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
	TIM1_TRG_COM_TIM11_IRQn    = 26,   /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	TIM1_CC_IRQn               = 27,   /*!< TIM1 Capture Compare Interrupt                                    */
	TIM2_IRQn                  = 28,   /*!< TIM2 global Interrupt                                             */
	TIM3_IRQn                  = 29,   /*!< TIM3 global Interrupt                                             */
	TIM4_IRQn                  = 30,   /*!< TIM4 global Interrupt                                             */
	I2C1_EV_IRQn               = 31,   /*!< I2C1 Event Interrupt                                              */
	I2C1_ER_IRQn               = 32,   /*!< I2C1 Error Interrupt                                              */
        I2C2_EV_IRQn               = 33,   /*!< I2C2 Event Interrupt                                              */
	I2C2_ER_IRQn               = 34,   /*!< I2C2 Error Interrupt                                              */
	SPI1_IRQn                  = 35,   /*!< SPI1 global Interrupt                                             */
	SPI2_IRQn                  = 36,   /*!< SPI2 global Interrupt                                             */
	USART1_IRQn                = 37,   /*!< USART1 global Interrupt                                           */
	USART2_IRQn                = 38,   /*!< USART2 global Interrupt                                           */
	EXTI15_10_IRQn             = 40,   /*!< External Line[15:10] Interrupts                                   */
	RTC_Alarm_IRQn             = 41,   /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
	OTG_FS_WKUP_IRQn           = 42,   /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
	DMA1_Stream7_IRQn          = 47,   /*!< DMA1 Stream7 Interrupt                                            */
	SDIO_IRQn                  = 49,   /*!< SDIO global Interrupt                                             */
	TIM5_IRQn                  = 50,   /*!< TIM5 global Interrupt                                             */
	SPI3_IRQn                  = 51,   /*!< SPI3 global Interrupt                                             */
	DMA2_Stream0_IRQn          = 56,   /*!< DMA2 Stream 0 global Interrupt                                    */
	DMA2_Stream1_IRQn          = 57,   /*!< DMA2 Stream 1 global Interrupt                                    */
	DMA2_Stream2_IRQn          = 58,   /*!< DMA2 Stream 2 global Interrupt                                    */
	DMA2_Stream3_IRQn          = 59,   /*!< DMA2 Stream 3 global Interrupt                                    */
	DMA2_Stream4_IRQn          = 60,   /*!< DMA2 Stream 4 global Interrupt                                    */
	OTG_FS_IRQn                = 67,   /*!< USB OTG FS global Interrupt                                       */
	DMA2_Stream5_IRQn          = 68,   /*!< DMA2 Stream 5 global interrupt                                    */
	DMA2_Stream6_IRQn          = 69,   /*!< DMA2 Stream 6 global interrupt                                    */
	DMA2_Stream7_IRQn          = 70,   /*!< DMA2 Stream 7 global interrupt                                    */
	USART6_IRQn                = 71,   /*!< USART6 global interrupt                                           */
	I2C3_EV_IRQn               = 72,   /*!< I2C3 event interrupt                                              */
	I2C3_ER_IRQn               = 73,   /*!< I2C3 error interrupt                                              */
	FPU_IRQn                   = 81,   /*!< FPU global interrupt                                              */
	SPI4_IRQn                  = 84,   /*!< SPI4 global Interrupt                                             */
	SPI5_IRQn                  = 85    /*!< SPI5 global Interrupt                                             */
} IRQn_e;


/**
 * @brief NVIC Priority numbers
 */
typedef enum
{
	NVIC_PRI0         = 0,
	NVIC_PRI1         = 1,
	NVIC_PRI2         = 2,
	NVIC_PRI3         = 3,
	NVIC_PRI4         = 4,
	NVIC_PRI5         = 5,
	NVIC_PRI6         = 6,
	NVIC_PRI7         = 7,
	NVIC_PRI8         = 8,
	NVIC_PRI9         = 9,
	NVIC_PRI10        = 10,
	NVIC_PRI11        = 11,
	NVIC_PRI12        = 12,
	NVIC_PRI13        = 13,
	NVIC_PRI14        = 14,
	NVIC_PRI15        = 15,
} PRI_e;


/***********************************************************
 *                   I2C
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for I2C
 */

typedef struct
{
    __IO uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
    __IO uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
    __IO uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
    __IO uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
    __IO uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
    __IO uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
    __IO uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
    __IO uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
    __IO uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
    __IO uint32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_RegDef_t;


/***********************************************************
 *                   SPI
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for SPI
 */

typedef struct
{
    __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
    __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
    __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
    __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_RegDef_t;


/**
 * @brief SPI CR1 register bit fields - anonymous enum usage
 */
enum
{
	SPI_CR1_CPHA     = 0,
	SPI_CR1_CPOL     = 1,
	SPI_CR1_MSTR     = 2,
	SPI_CR1_BR       = 3,      /* BR[5:3] */
	SPI_CR1_SPE      = 6,
	SPI_CR1_LSBFIRST = 7,
	SPI_CR1_SSI      = 8,
	SPI_CR1_SSM      = 9,
	SPI_CR1_RXONLY   = 10,
	SPI_CR1_DFF      = 11,
	SPI_CR1_CRCNEXT  = 12,
	SPI_CR1_CRCEN    = 13,
	SPI_CR1_BIDIOE   = 14,
	SPI_CR1_BIDIMODE = 15,
};


/**
 * @brief SPI CR2 register bit fields - anonymous enum usage
 */
enum
{
	SPI_CR2_RXDMAEN  = 0,
	SPI_CR2_TXDMAEN  = 1,
	SPI_CR2_SSOE     = 2,
	SPI_CR2_FRF      = 4,
	SPI_CR2_ERRIE    = 5,
	SPI_CR2_RXNEIE   = 6,
	SPI_CR2_TXEIE    = 7,
};


/**
 * @brief SPI SR register bit fields - anonymous enum usage
 */
enum
{
	SPI_SR_RXNE     = 0,
	SPI_SR_TXE      = 1,
	SPI_SR_CHSIDE   = 2,
	SPI_SR_UDR      = 3,
	SPI_SR_CRCERR   = 4,
	SPI_SR_MODF     = 5,
	SPI_SR_OVR      = 6,
	SPI_SR_BSY      = 7,
	SPI_SR_FRE      = 8,
};


/**
 * @brief SPIx Definition (Peripheral base addresses type casted to xxxRegDef_t)
 */
#define SPI1                    ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2                    ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3                    ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4                    ((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5                    ((SPI_RegDef_t *)SPI5_BASEADDR)


/***********************************************************
 *                   SYSCFG
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__IO uint32_t MEMRMP;       /*!< SYSCFG Register,   Addr offset: 0x00*/
	__IO uint32_t PMC;          /*!< short description, Addr offset: 0x04*/
	__IO uint32_t EXTICR[4];    /*!< short description, Addr offset: 0x08 - 0x14*/
	uint32_t      RESERVED[2];  /*!< short description, Addr offset: 0x18 - 0x1C*/
	__IO uint32_t CMPCR;        /*!< short description, Addr offset: 0x20*/
} SYSCFG_RegDef_t;


/**
 * @brief EXTI Definition (Peripheral base addresses type casted to xxxRegDef_t)
 */
#define SYSCFG                   ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)



/***********************************************************
 *                   USART
 ***********************************************************/
/**
 * @brief Peripheral register definition structure for USART
 */

typedef struct
{
    __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
    __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
    __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
    __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
    __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
    __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
    __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_RegDef_t;



/***********************************************************
 *                   Clock Enable Macros
 ***********************************************************/
/**
 * @brief Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1U << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1U << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1U << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1U << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1U << 4))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1U << 7))


/**
 * @brief Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()         (RCC->APB1ENR |= (1U << 21))
#define I2C2_PCLK_EN()         (RCC->APB1ENR |= (1U << 22))
#define I2C3_PCLK_EN()         (RCC->APB1ENR |= (1U << 23))


/**
 * @brief Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()         (RCC->APB2ENR |= (1U << 12))
#define SPI2_PCLK_EN()         (RCC->APB1ENR |= (1U << 14))
#define SPI3_PCLK_EN()         (RCC->APB1ENR |= (1U << 15))
#define SPI4_PCLK_EN()         (RCC->APB2ENR |= (1U << 13))
#define SPI5_PCLK_EN()         (RCC->APB2ENR |= (1U << 20))

/**
 * @brief Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()       (RCC->APB2ENR |= (1U << 4))
#define USART2_PCLK_EN()       (RCC->APB1ENR |= (1U << 17))
#define USART6_PCLK_EN()       (RCC->APB2ENR |= (1U << 5))

/**
 * @brief Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()       (RCC->APB2ENR |= (1U << 14))



/***********************************************************
 *                   Clock Disable Macros
 ***********************************************************/
/**
 * @brief Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 4))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1U << 7))


/**
 * @brief Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()         (RCC->APB1ENR &= ~(1U << 21))
#define I2C2_PCLK_DI()         (RCC->APB1ENR &= ~(1U << 22))
#define I2C3_PCLK_DI()         (RCC->APB1ENR &= ~(1U << 23))


/**
 * @brief Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()         (RCC->APB2ENR &= ~(1U << 12))
#define SPI2_PCLK_DI()         (RCC->APB1ENR &= ~(1U << 14))
#define SPI3_PCLK_DI()         (RCC->APB1ENR &= ~(1U << 15))
#define SPI4_PCLK_DI()         (RCC->APB2ENR &= ~(1U << 13))
#define SPI5_PCLK_DI()         (RCC->APB2ENR &= ~(1U << 20))

/**
 * @brief Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()       (RCC->APB2ENR &= ~(1U << 4))
#define USART2_PCLK_DI()       (RCC->APB1ENR &= ~(1U << 17))
#define USART6_PCLK_DI()       (RCC->APB2ENR &= ~(1U << 5))

/**
 * @brief Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()       (RCC->APB2ENR &= ~(1U << 14))


/**
 * @brief GPIO port macro - !!! CONVERT TO C FUNCTION LATER !!!
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 : \
		                   (x == GPIOB) ? 1 : \
		                   (x == GPIOC) ? 2 : \
		                   (x == GPIOD) ? 3 : \
		                   (x == GPIOE) ? 4 : \
		                   (x == GPIOH) ? 7 : 0)



#endif /* STM32F411XX_H_ */
