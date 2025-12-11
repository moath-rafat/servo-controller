#ifndef STM32F4XX_H_
#define STM32F4XX_H_

#include <stdint.h>

/* ================================================================
 *                    BASE ADDRESSES FOR MEMORY
 * ================================================================ */
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR 0x1FFF0000U

#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

/* ================================================================
 *                    AHB1 PERIPHERALS
 * ================================================================ */
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

/* ================================================================
 *                    APB1 PERIPHERALS
 * ================================================================ */
#define TIM2_BASEADDR (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR (APB1PERIPH_BASEADDR + 0x0C00)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0X3C00)
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)

/* ================================================================
 *                    APB2 PERIPHERALS
 * ================================================================ */
#define TIM1_BASEADDR (APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR (APB2PERIPH_BASEADDR + 0x0400)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR (APB2PERIPH_BASEADDR + 0x2000)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0X3000)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)

/* ================================================================
 *                    PERIPHERAL STRUCTURES
 * ================================================================ */

/* ---------------- GPIO ---------------- */
typedef struct
{
    volatile uint32_t MODER;   /* Mode register                     */
    volatile uint32_t OTYPER;  /* Output type register              */
    volatile uint32_t OSPEEDR; /* Output speed register             */
    volatile uint32_t PUPDR;   /* Pull-up/Pull-down register        */
    volatile uint32_t IDR;     /* Input data register               */
    volatile uint32_t ODR;     /* Output data register              */
    volatile uint32_t BSRR;    /* Bit set/reset register            */
    volatile uint32_t LCKR;    /* Configuration lock register       */
    volatile uint32_t AFRL;    /* Alternate function low register   */
    volatile uint32_t AFRH;    /* Alternate function high register  */
} GPIO_RegDef_t;

/* ---------------- RCC ---------------- */
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    uint32_t RESERVED2[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    uint32_t RESERVED4[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
} RCC_RegDef_t;

/* ---------------- EXTI ---------------- */
typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_RegDef_t;

/* ---------------- SYSCFG ---------------- */
typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    uint32_t RESERVED1[2];
    volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;

/* ---------------- TIMER ---------------- */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_RegDef_t;

/* ---------------- SPI ---------------- */
typedef struct
{
    volatile uint32_t SPI_CR1;
    volatile uint32_t SPI_CR2;
    volatile uint32_t SPI_SR;
    volatile uint32_t SPI_DR;
    volatile uint32_t SPI_CRCPR;
    volatile uint32_t SPI_RXCRCR;
    volatile uint32_t SPI_TXCRCR;
    volatile uint32_t SPI_I2SCFGR;
    volatile uint32_t SPI_I2SPR;
} SPI_RegDef_t;

/* ---------------- I2C ---------------- */
typedef struct
{
    volatile uint32_t I2C_CR1;
    volatile uint32_t I2C_CR2;
    volatile uint32_t I2C_OAR1;
    volatile uint32_t I2C_OAR2;
    volatile uint32_t I2C_DR;
    volatile uint32_t I2C_SR1;
    volatile uint32_t I2C_SR2;
    volatile uint32_t I2C_CCR;
    volatile uint32_t I2C_TRISE;
    volatile uint32_t I2C_FLTR;
} I2C_RegDef_t;

/* ---------------- UART ---------------- */
typedef struct
{
    volatile uint32_t USART_SR;
    volatile uint32_t USART_DR;
    volatile uint32_t USART_BRR;
    volatile uint32_t USART_CR1;
    volatile uint32_t USART_CR2;
    volatile uint32_t USART_CR3;
    volatile uint32_t USART_GTPR;
} USART_RegDef_t;

/* ---------------- NVIC ---------------- */
typedef struct
{
    volatile uint32_t ISER[8];
    uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];
    uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];
    uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];
    uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];
    uint32_t RESERVED4[56];
    volatile uint8_t IPR[240];
    uint32_t RESERVED5[644];
    volatile uint32_t STIR;
} NVIC_RegDef_t;

/* ================================================================
 *                    PERIPHERAL DEFINITIONS (MACROS)
 * ================================================================ */
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define USART4 ((USART_RegDef_t *)USART4_BASEADDR)
#define USART5 ((USART_RegDef_t *)USART5_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)

#define TIM1 ((TIM_RegDef_t *)TIM1_BASEADDR)
#define TIM2 ((TIM_RegDef_t *)TIM2_BASEADDR)
#define TIM3 ((TIM_RegDef_t *)TIM3_BASEADDR)
#define TIM4 ((TIM_RegDef_t *)TIM4_BASEADDR)
#define TIM5 ((TIM_RegDef_t *)TIM5_BASEADDR)
#define TIM8 ((TIM_RegDef_t *)TIM8_BASEADDR)

#define NVIC ((NVIC_RegDef_t *)0xE000E100U)

/* ================================================================
 *                    HELPER MACROS
 * ================================================================ */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#define INPUT   0
#define OUTPUT  1
#define AF      2
#define ANALOG  3

#define PULL_UP 1
#define PULL_DO 2

#endif /* STM32F4XX_H_ */
