

#ifndef INC_STM32F411RE_USART_DRIVER_H_
#define INC_STM32F411RE_USART_DRIVER_H_


#include "STM32F411RE.h"

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;



typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t   USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;




 //Possible options for USART_Mode

#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2


// Possible options for USART_Baud

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000



#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0


#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1


#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3



#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


#define USART_SR_PE		0
#define USART_SR_FE		1
#define USART_SR_NF		2
#define USART_SR_ORE	3
#define USART_SR_IDLE	4
#define USART_SR_RXNE	5
#define USART_SR_TC		6
#define USART_SR_TXE	7
#define USART_SR_LBD	8
#define USART_SR_CTS	9

//APP STATUS
#define USART_EVENT_TX_CMPLT     0
#define USART_EVENT_RX_CMPLT     1
#define USART_EVENT_CTS          2
#define USART_EVENT_IDLE         3
#define USART_EVENT_ORE          4

// Error events
#define USART_ERREVENT_FE        5
#define USART_ERREVENT_NE        6
#define USART_ERREVENT_ORE       7


#define USART_READY        0
#define USART_BUSY_IN_RX   1
#define USART_BUSY_IN_TX   2

 // Peripheral Clock setup

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


 // Init and De-init

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);



 // Data Send and Receive

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


 // IRQ Configuration and ISR handling

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);


 // Other Peripheral Control APIs

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);


 // Application callback

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPCLK1Value(void);




#endif /* INC_STM32F411RE_USART_DRIVER_H_ */
