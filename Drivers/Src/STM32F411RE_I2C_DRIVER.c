#include "STM32F411RE_I2C_DRIVER.h"

#include "STM32F411RE_USART_DRIVER.h"

#define READ 1
#define WRITE 0

extern uint16_t AHB_PreScaler[8];
extern uint8_t APB1_PreScaler[4];


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t isRead);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t  *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t  *pI2CHandle);


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}




void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ENABLE ACK IN CR1
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//CONFIGURE FREQFIELD IN CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Device OWN ADDRESS SLAVEMODE
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR register calculations
	uint16_t ccr_value =0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSPEED <= I2C_SCL_SPEED_SM)
	{
		//STANDARD MODE
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSPEED));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//FAST MODE
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle== I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSPEED));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSPEED));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE CONFIG
	if(pI2CHandle->I2C_Config.I2C_SCLSPEED <= I2C_SCL_SPEED_SM)
		{
			//STANDARD MODE
			tempreg = (RCC_GetPCLK1Value() / 1000000U)+1;
		}else
		{
			//FAST MODE
			tempreg = ((RCC_GetPCLK1Value() * 300) /1000000000U)+1;
		}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

//SEND DATA
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//FIRST start condition,
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//CHECK IF START IS COMPLETED BY SB FLAG IN SR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//SEND The Address of the slave with r/w bit set to w(0) 1 byte total
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr,WRITE);
	//CHECK ADDRESS PHASE COMPLETE ADDR BIT IN SR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));
	//CLEAR ADDR
	I2C_ClearADDRFlag(pI2CHandle);
	//SEND DATA UNTIL LEN =0
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//when len =0 wait for TXE 1 AND BTF 1 before STOP CONDITION
	//Note: TXE=1, BTF = 1, MEANS THAT BOTH SR AND DR ARE EMPTY AND
	//NEXT TRANSMISSSION SHOULD BEGIN WHEN BTF=1 and SCL WILL BE STRETCHED PULLEDLOW
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));



	//GENERATE THE STOP CONDITION TO TERMINATE, automatically CLEARS BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//FIRST start condition,
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//CHECK IF START IS COMPLETED BY SB FLAG IN SR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//SEND The Address of the slave with r/w bit set to w(0) 1 byte total
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr,READ);
	//CHECK ADDRESS PHASE COMPLETE ADDR BIT IN SR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//SEND DATA UNTIL LEN =0
	if(Len==1)
	{
		//DISABLE ACKING LEN = 1
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
		//GENERATE STOP CONDITION
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//CLEAR ADDR FLAG
		I2C_ClearADDRFlag(pI2CHandle);
		//WAIT UNTIL RXNE BECOMES 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
		//READ DATA IN TO BUFFER
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len>1)
	{
		//CLEAR ADDR FLAG
		I2C_ClearADDRFlag(pI2CHandle);
		//READ DATA UNTIL LEN BECOME 1
		for(uint32_t i = Len; i>0; i--)
		{
			//WAIT UNTIL RXNE BECOMES 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
			if(i==2)
			{
				//DISABLE ACKING when i =2
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
				//GENERATE STOPP
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//READ DATA IN TO BUFFER
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//increment buffer addr
			pRxBuffer++;
		}
	}

//RENABLE THE ACKING BECAUSE WE DISABLED IT
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}



uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;


		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			// Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}










void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//ISER0
				*NVIC_ISER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber <= 63)
			{
				//ISER1
				*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			}
			else
			{
				//ISER2
				*NVIC_ISER2 |= (1 << (IRQNumber % 64));
			}
		}
		else if (EnorDi == DISABLE)
		{
			if(IRQNumber <= 31)
			{
				//ICER0
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber <= 63)
			{
				//ICER1
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}
			else
			{
				//ICER2
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			}
		}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//first step FIND THE IPR REGISTER
	uint8_t iprx = IRQNumber / 4;
	//which section of the register
	uint8_t iprx_section = IRQNumber % 4;

	//DUE to the architecture of this MCU, the 4 RIGHT BITS OF EACH BYTE is not utilized, so we have to shift accordingly
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount);
}




//uint32_t RCC_GetPLLOutputClock()
//{
//	return;
//}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 &(1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 &(1<<I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_SB);
	//interrupt generated by SB event
	if(temp1 &&temp3)
	{
		//SB FLAG is SET, interrupt caused by SB even only in master MODE,
		//start successful, excute address phase
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX)
		{
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,WRITE);
		}else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
		{
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,READ);
		}
	}

	// Handle For interrupt generated by ADDR event
	temp3=pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_ADDR);
	if(temp1&&temp3)
	{
		//CLEAR addr flag to stop stretching
		I2C_ClearADDRFlag(pI2CHandle);

	}

	// interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_BTF);
	if(temp1&&temp3)
	{
		//BTF IS SET
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure TXE IS SET
			if(pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_TXE))
			{
				//BTF AND TXE ARE SET, CLOSE TRANSMITION,RESET THE HANDLE AND NOTIFIY THAT ITS COMPLETE
				//make sure TXLEN = 0
				if(pI2CHandle->TxLen == 0)
				{
					//stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//RESET MEMBER ELEMENTS IN HANDLE
					I2C_CloseSendData(pI2CHandle);

					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
				;
		}
	}
	//interrupt generated by STOPF event, SLAVEMODE
	temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_STOPF);
	if(temp1&&temp3)
	{
		//STOPF IS SET, now clear it by reading SR and write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);

	}

	// interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_TXE);
	if(temp1&&temp2&&temp3)
	{
		//only in masterMODE
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
		{
		//TXE IS SET, start data transmission
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
	}

	//interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_RXNE);
	if(temp1&&temp2&&temp3)
		{
			if(pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_MSL))
			{
			//RXNE IS SET
				if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
				{
					//DATA RECEPTION
					I2C_MasterHandleRXNEInterrupt(pI2CHandle);
				}
			}
		}
}




void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	//Check for Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		// clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		// notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	//Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		// Clear the arbitration lost flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//Check for ACK failure  error

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		// This is ACK failure error
		// Clear the ACK failure flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//Check  for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		// Clear the Overrun/Underrun flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	//Check for Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		// Clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t  *pI2CHandle)
{
	if(pI2CHandle->RxSize ==1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize >1)
	{
		if(pI2CHandle->RxLen==2)
		{
			//DISABLE ACK
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}
		//READ DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen==0)
	{
		//CLOSE RECEPTION AND CALLBACK
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}


static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t  *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//load the data into DR deccrement TX and increment buffer
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

//uint32_t RCC_GetPCLK1Value(void)
//{
//	uint32_t PCLK1,SYSTEMCLK;
//	uint8_t CLKSRC,temp,AHBP,APB1P;
//
//	CLKSRC = (RCC->CFGR >> 2) & 0x3;
//
//	if(CLKSRC == 0)
//	{
//		SYSTEMCLK = 16000000;
//	}else if(CLKSRC == 1)
//	{
//		SYSTEMCLK = 8000000;
//	}else if(CLKSRC == 2)
//	{
//		//SYSTEMCLK = RCC_GetPLLOutputClock();
//	}
//
//
//	//AHB
//	temp = ((RCC->CFGR >> 4) & 0xF);
//
//	if(temp < 8)
//	{
//		AHBP = 1;
//	}else
//	{
//		AHBP = AHB_PreScaler[temp-8];
//	}
//	//APB1
//	temp = ((RCC->CFGR >> 10) & 0x7);
//
//	if(temp < 4)
//	{
//		APB1P = 1;
//	}else
//	{
//		APB1P = APB1_PreScaler[temp-4];
//	}
//
//	PCLK1 = (SYSTEMCLK/AHBP)/APB1P;
//
//
//
//	return PCLK1;
//}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}



static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t isRead)
{
	SlaveAddr = SlaveAddr <<1;
	if(isRead)
	{
	//SET the 0th bit for READ OPERATION
		SlaveAddr |= 1;
	}
	else{
	//Clear the 0th bit for WRITE OPERATION
	SlaveAddr &= ~(1);
	}
	pI2Cx->DR = SlaveAddr;

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//CHECK DEVICE MODE
	if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))//MASTER
	{
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize ==1)
			{
				//DISABLE ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
				//CLEAR ADDR
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}else
		{
			//CLEAR ADDR
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}else //SLAVE
	{
		//CLEAR ADDR
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//ENABLE
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		//DISABLE
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//DISABLE ITBUFEN CONTROLBIT
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
	//DISABLE ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);
	//RESET HANDLE
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen=0;
	pI2CHandle->RxSize=0;
	//ENABLE ACK
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//DISABLE ITBUFEN CONTROLBIT
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
	//DISABLE ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
}





