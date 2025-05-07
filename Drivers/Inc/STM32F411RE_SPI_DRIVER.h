

#ifndef INC_STM32F411RE_SPI_DRIVER_H_
#define INC_STM32F411RE_SPI_DRIVER_H_


#include "STM32F411RE.h"

typedef struct {
    uint8_t SPI_DeviceMode;            // SPI mode (Master or Slave)
    uint8_t SPI_BusConfig;            // FULL DUPLEX/HALF DUPLEX/SIMPLEX
    uint8_t SPI_SclkSpeed;     		 // clock speed
    uint8_t SPI_DFF;       		    // reg size, 8/16
    uint8_t SPI_CPOL;  		       // polarity
    uint8_t SPI_CPHA;             // clk phase edge selection
    uint8_t SPI_SSM;             // Slave select mode (Hardware or Software)
} SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;  //this holds the base address of the SPI
	SPI_Config_t SPI_Config;
	uint8_t 	 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	  TxLen;
	uint32_t	  RxLen;
	uint8_t		  TxState;
	uint8_t		  RxState;
}SPI_Handle_t;


//SPI STATUS
#define SPI_READY 				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2
//SPI APPLICATION EVENT
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

//SPI MODE

#define		SPI_DEVICE_MODE_SLAVE		0        //SLAVE
#define		SPI_DEVICE_MODE_MASTER		1



//BUS CONFIGS

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3


//TRANSMITION SIZE DFF

#define		DATA_SIZE_8		0 		// 8 BITS
#define		DATA_SIZE_16	1		//16 BITS

//CLK  POLARITY CPOL

#define		CPOL_LOW			0		// SCK IS SET TO 0 WHEN IDLE
#define		CPOL_HI				1		//SCK IS SET TO 1 WHEN IDLE

//CLK PHASE

#define		 CPHA_LOW			0
#define		 CPHA_HIGH			1


// PRESCALARS FOR SCK

#define	    SCLK_DIV2				0		// FPCLK / 2  == SCLK
#define		SCLK_DIV4				1		// FPCLK / 4 == SCLK
#define		SCLK_DIV8				2		//FPCLK /8  == SCLK
#define		SCLK_DIV16				3		// FPCLK / 16 == SCLK
#define		SCLK_DIVE32				4		//FPCLK /32  == SCLK
#define		SCLK_DIV64				5		//FPCLK /64  ==SCLK
#define		SCLK_DIV128				6		// FPCLK /128  ==SCLK
#define		SCLK_DIV256				7		//FPCLK  / 256  == SCLK






// SLAVE SELECT MODES

#define		SPI_SSM_EN			1		// software slave management disabled
#define		SPI_SSM_DI			0		//software slave management enabled



//SPI RELATED STATUS FLAGS DEF
#define SPI_TXE_FLAG 			(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG			(1 <<SPI_SR_BSY)




//SPI CLOCK CONTROL

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


//INIT AND DEINIT
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


//DATA SEND AND RECEIVE



void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// WITH INTERUPTS

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *SPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ CONFIG AND ISR HANDLING

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// OTHER PERIPHERAL CONTROL API

void SPI_PeripheralControl(SPI_RegDef_t * pSPIx, uint8_t EnorDi);



void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

//APLICATION CALLBACK
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);



#endif /* INC_STM32F411RE_SPI_DRIVER_H_ */
