//THIS IS STM32F411RE.h

#ifndef INC_STM32F411RE_H_
#define INC_STM32F411RE_H_



#include <stdint.h>
#include<stddef.h>
#define __vol volatile
#define	__weak	 __attribute__((weak))

// ---------------PROCESSOR SPECIFIC DETAILS

//-- NVIC REGISTERS

				// ISER
#define NVIC_ISER0	((__vol uint32_t*)0xE000E100)
#define NVIC_ISER1	((__vol uint32_t*)0xE000E104)
#define NVIC_ISER2	((__vol uint32_t*)0xE000E108)
#define NVIC_ISER3	((__vol uint32_t*)0xE000E10C)

				//ICER
#define NVIC_ICER0 	((__vol uint32_t*)0XE000E180)
#define NVIC_ICER1 	((__vol uint32_t*)0XE000E184)
#define NVIC_ICER2 	((__vol uint32_t*)0XE000E188)
#define NVIC_ICER3 	((__vol uint32_t*)0XE000E18C)

				//PRIORITY REGISTERS REGISTER CALCULATION
#define NVIC_PR_BASE_ADDR	((__vol uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4    //MCU SPECIFIC FOR M4 it is 4 BITS are not utilized

/* BASE ADDRESSES OF MEMORIES */

#define FLASH_BASEADDR  0x08000000U
#define SRAM1_BASEADDR  0x20000000U
#define ROM             0x1FFF0000U
/*---------------------------------------------------------*/
#define SRAM            SRAM1_BASEADDR


/* BASE ADDRESSES OF BUS DOMAINS */
#define PERIPH_BASEADDR  0x40000000U
#define APB1PERIPH_BASEADDR  PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR  0x40010000U
#define AHB1PERIPH_BASEADDR  0x40020000U
#define AHB2PERIPH_BASEADDR  0x50000000U


/* PERIPHRAL ADDRESSES WHICH ARE CONNECTED TO AHB1 */

				//GPIO
#define GPIOA_BASEADDR  (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR	(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR  (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR	(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR  (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR  (AHB1PERIPH_BASEADDR + 0x1C00)


				//RCC
#define RCC_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x3800)

/*---------------------------------------------------------*/

/* PERIPHRAL ADDRESSES WHICH ARE CONNECTED TO APB1 */

                //I2C
#define	I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define	I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define	I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

               //SPI
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define	SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)

				//USART
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)


/* -----------------------------------------------------------------------*/

/* PERIPHRAL ADDRESSES WHICH ARE CONNECTED TO APB2 */

               //SPI
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define	SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)
#define	SPI5_BASEADDR (APB2PERIPH_BASEADDR + 0x5000)

				//USART
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)

                //SYSCFG_BASE
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)

				//EXTI_BASE
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)

/*---------------------------------------------------------------*/

			/*Peripheral Register definition FOR GPIO*/

typedef struct {
	__vol uint32_t MODER;		//MODE OF THE GPIO ADDRESS OFFSET: 0x00
	__vol uint32_t OTYPER;		//TYPE OF OUTPUT, ONLY VIABLE FOR OUTPUT MODER, PUSHPULL OR OPENDRAIN, ADDRESS OFFSET: 0x04
	__vol uint32_t OSPEEDR;		//OUTPUT SPEED, 00 LOW, 01MEDIUM,10FAST,11HIGH depending on voltage supplied VDD or capacitive loads
	__vol uint32_t PUPDR;        //DETERMINES IF ITS PULL UP 01/PULL DOWN11/ NEITHER 00(OFF) FLOATING//AND 11 RESERVED NO USE
	__vol uint32_t IDR;			//READ ONLY, USED FOR reading data from input,
	__vol uint32_t ODR;			//READ ONLY, USED TO READ OUTGOING DATA AT A CERTAIN POINT OF TIME
	__vol uint32_t BSRR;			//port bit set/reset register
	__vol uint32_t LCKR;			//port configuration lock register
	__vol uint32_t AFR[2];		//alternate function low register AFR[0]=pin 0-7 AFR[1]=ping 8-15
}GPIO_RegDef_t;

typedef struct{
	__vol uint32_t CR;                   //OFFSET 0x00
	__vol uint32_t PLLVFGR;				//OFFSET 0x04
	__vol uint32_t CFGR;					//OFFSET 0x08
	__vol uint32_t CIR;					//OFFSET 0x0C
	__vol uint32_t AHB1RSTR;				//OFFSET 0x10
	__vol uint32_t AHB2RSTR;				//OFFSET 0x14
	uint32_t RESERVED0;					//OFFSET 0x18
	uint32_t RESERVED1;					//OFFSET 0x1C
	__vol uint32_t APB1RSTR;				//OFFSET 0x20
	__vol uint32_t APB2RSTR;				//OFFSET 0x24
	uint32_t RESERVED2;					//OFFSET 0x28
	uint32_t RESERVED3;					//OFFSET 0x2C
	__vol uint32_t AHB1ENR;				//OFFSET 0x30
	__vol uint32_t AHB2ENR;				//OFFSET 0x34
	uint32_t RESERVED4;					//OFFSET 0x38
	uint32_t RESERVED5;					//OFFSET 0x3C
	__vol uint32_t APB1ENR;				//OFFSET 0x40
	__vol uint32_t APB2ENR;				//OFFSET 0x44
	uint32_t RESERVED6;					//OFFSET 0x48
	uint32_t RESERVED7;					//OFFSET 0x4C
	__vol uint32_t AHB1LPENR;			//OFFSET 0x50
	__vol uint32_t AHB2LPENR;			//OFFSET 0x54
	uint32_t RESERVED8;					//OFFSET 0x58
	uint32_t RESERVED9;					//OFFSET 0x5C
	__vol uint32_t APB1LPENR;			//OFFSET 0x60
	__vol uint32_t APB2LPENR;			//OFFSET 0x64
	uint32_t RESERVED10;					//OFFEST 0x68
	uint32_t RESERVED11;					//OFFSET 0x6C
	__vol uint32_t BDCR;					//OFFSET 0x70
	__vol uint32_t CSR;					//OFFSET 0x74
	uint32_t RESERVED12;					//OFFSET 0x78
	uint32_t RESERVED13;					//OFFSET 0x7C
	__vol uint32_t SSCGR;				//OFFSET 0x80
	__vol uint32_t PLLI2SCFGR;			//OFFSET 0x84
	uint32_t RESERVED14;					//OFFSET 0x88
	__vol uint32_t DCKCFGR;				//OFFSET 0x8C
}RCC_RegDef_t;


typedef struct {
	__vol uint32_t CR1;             //OFFSET 0x00 ctrl REG 1
	__vol uint32_t CR2;				//OFFSET 0x04 ctrl REG 2
	__vol uint32_t SR;				//OFFSET 0x08 STATUS REG
	__vol uint32_t DR;				//OFFSET 0x0C DATA REG
	__vol uint32_t CRCPR;			//OFFSET 0x10 CRC POLYNOMIAL REG
	__vol uint32_t RXCRCR;			//OFFSET 0x14 RX CRC REGISTER
	__vol uint32_t TXCRCR;			//OFFSET 0x18 TX CRC REG
	__vol uint32_t I2SCFGR;			//OFFSET 0x1C I2S config reg
	__vol uint32_t I2SPR;			//OFFSET 0x20 I2S PRESCALAR REGISTER
}SPI_RegDef_t;

typedef struct {
	__vol uint32_t MEMRMP;			//OFFSET 0X00
	__vol uint32_t PMC;				//OFFSET 0X04
	__vol uint32_t EXTICR[4];		//OFFSET 0X08 - 0X014
	__vol uint32_t RESERVED1;		//OFFSET 0X18
	__vol uint32_t RESERVED2;		//OFFSET 0X1C
	__vol uint32_t EXTICMPCR;		//OFFSET 0X20
}SYSCFG_RegDef_t;


typedef struct {
	__vol uint32_t IMR;				//OFFSET 0X00
	__vol uint32_t EMR;				//OFFSET 0X04
	__vol uint32_t RTSR;			//OFFSET 0X08
	__vol uint32_t FTSR;			//OFFSET 0X0C
	__vol uint32_t SWIER;			//OFFSET 0X10
	__vol uint32_t PR;				//OFFSET 0X14
}EXTI_RegDef_t;



typedef struct {
	__vol uint32_t CR1;   //CONTROL REGISTER 1
	__vol uint32_t CR2;	  //CONTROL REGISTER 2
	__vol uint32_t OAR1;  // OWN ADDRESS REGISTER 1
	__vol uint32_t OAR2; // OWN ADDRESS REGISTER 2
	__vol uint32_t DR;   //DATA REGISTER
	__vol uint32_t SR1;  //STATUS REGISTER 1
	__vol uint32_t SR2;  //STATUS REGISTER 2
	__vol uint32_t CCR;  //CLOCK CONTROL REGISTER
	__vol uint32_t TRISE; //
	__vol uint32_t FLTR;  // NOISE FILTERS
}I2C_RegDef_t;




typedef struct{
	__vol uint32_t SR;
	__vol uint32_t DR;
	__vol uint32_t BRR;
	__vol uint32_t CR1;
	__vol uint32_t CR2;
	__vol uint32_t CR3;
	__vol uint32_t GTPR;
}USART_RegDef_t;
/*                 PERIPHERAL DEFINITION( PERIPHERAL BASE ADDRESSES TYPECASTED TO XXX_REGDEF_T)        */


#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5 		((SPI_RegDef_t*)SPI5_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*------------------------------------------------------------------------------------*/

//CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS

#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1<<7))


//CLOCK ENABLE MACROS FOR I2C PERIPHERALS

#define I2C1_PCLK_EN()	 (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()	 (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1<<23))

//CLOCK ENABLE MACROS FOR SPI PERIPHERALS

             //APB1

#define SPI2_PCLK_EN()	 (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1<<15))

             //APB2

#define SPI1_PCLK_EN()	 (RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()	 (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()	 (RCC->APB2ENR |= (1<<20))


//CLOCK ENABLE MACROS FOR UART/USART PERIPHERALS

					//APB1
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1<<17))


					//APB2
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1<<5))


//CLOCK ENABLE FOR SYSCFG PERIPHERAL

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))

/*======================================================================================*/

//PERIPHERAL CLOCK DISABLE MACROS

//CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<7))


//CLOCK DISABLE MACROS FOR I2C PERIPHERALS

#define I2C1_PCLK_DI()	 (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()	 (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<23))



//CLOCK DISABLE MACROS FOR SPI PERIPHERALS

             //APB1

#define SPI2_PCLK_DI()	 (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<15))

             //APB2

#define SPI1_PCLK_DI()	 (RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()	 (RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()	 (RCC->APB2ENR &= ~(1<<20))


//CLOCK DISABLE MACROS FOR UART/USART PERIPHERALS

					//APB1
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<17))


					//APB2
#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1<<5))


//CLOCK DISABLE FOR SYSCFG PERIPHERAL

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))


//MACROS TO RESET GPIOX PERIPHERAL

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

//MACROS TO RESET SPIX PERIPHERAL
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)
#define SPI5_REG_RESET()	do{(RCC->APB2RSTR |= (1<<20)); (RCC->APB2RSTR &= ~(1<<20));}while(0)

//MACRO TO RESET I2Cx PERIPHERAL
#define I2C1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<21)); (RCC->APB2RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()	do{(RCC->APB2RSTR |= (1<<22)); (RCC->APB2RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()	do{(RCC->APB2RSTR |= (1<<23)); (RCC->APB2RSTR &= ~(1<<23));}while(0)

#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART6_REG_RESET()	do{(RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));}while(0)
//RETURN PORT CODE FOR GIVEN GPIO BASE ADDR
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOH) ? 7 :0 )
//GENERIC MACROS
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

//BIT POSITIONS IN SPI CR1

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//BIT POSITIONS IN SPI CR2

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//BIT POSITIONS IN SPI SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR  		6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


//BIT POSITIONS IN I2C CR1

#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

//BIT POSITIONS IN I2C CR2

#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

//BIT POSITIONS IN I2C SR1

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

//BIT POSITIONS IN I2C SR2

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

//BIT POSITIONS IN I2C CCR
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15


//BIT POSITIONS FOR USART
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9
#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE 			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11
#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8


//IRQ------------------------
//GPIO-----------------------
#define IRQ_NO_EXTI0		6
#define	IRQ_NO_EXTI1		7
#define	IRQ_NO_EXTI2		8
#define	IRQ_NO_EXTI3		9
#define	IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI5_9		23
#define IRQ_NO_EXTI10_15	40
//SPI------------------------
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
//I2C------------------------
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73


//IRQ PRIORITY
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI15	15








#endif
