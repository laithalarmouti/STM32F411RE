#ifndef INC_STM32F411RE_I2C_DRIVER_H_
#define INC_STM32F411RE_I2C_DRIVER_H_

#include "STM32F411RE.h"

typedef struct{
	uint32_t I2C_SCLSPEED;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t	Sr;
}I2C_Handle_t;


//I2C STATUS
#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2


//SCLSPEED
#define I2C_SCL_SPEED_SM	 100000
#define I2C_SCL_SPEED_FM4K 	 400000
#define I2C_SCL_SPEED_FM2K   200000

//ACKControl
#define I2C_ACK_ENABLE		1 //ACK ENABLE
#define I2C_ACK_DISABLE		0 // NO ACK, DEFAULT

//DUTYCYCLE
#define I2C_FM_DUTY_2		0 // SCL LOW = SCL HIGH 50%
#define I2C_FM_DUTY_16_9	1 // SCL LOW is 16 UNITS LONG HI is 9

//FLAGS
#define I2C_FLAG_TXE			(1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE			(1<<I2C_SR1_RXNE)
#define I2C_FLAG_SB				(1<<I2C_SR1_SB)
#define I2C_FLAG_OVR			(1<<I2C_SR1_OVR)
#define I2C_FLAG_AF				(1<<I2C_SR1_AF)
#define I2C_FLAG_ARLO			(1<<I2C_SR1_ARLO)
#define I2C_FLAG_BERR			(1<<I2C_SR1_BERR)
#define I2C_FLAG_STOPF			(1<<I2C_SR1_STOPF)
#define I2C_FLAG_ADD10			(1<<I2C_SR1_ADD10)
#define I2C_FLAG_BTF			(1<<I2C_SR1_BTF)
#define I2C_FLAG_ADDR			(1<<I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT		(1<<I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR RESET
#define I2C_ENEABLE_SR SET

//I2C APPLICATION MACROS

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO		4
#define I2C_ERROR_AF		5
#define I2C_ERROR_OVR		6
#define I2C_ERROR_TIMEOUT   7





// FUNCTION PROTOTYPES

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

//SEND DATA
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr);

//RECEIVE DATA
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr);



//SEND DATA INTERRUPT
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

//RECEIVE DATA INTERRUPT
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

// IRQ CONFIG AND ISR HANDLING
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);



//APLICATION CALLBACK
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);



uint32_t RCC_GetPCLK1Value(void);



#endif /* INC_STM32F411RE_I2C_DRIVER_H_ */
