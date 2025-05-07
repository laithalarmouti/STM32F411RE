#include "STM32F411RE_USART_DRIVER.h"



uint16_t AHB_PreScaler[8]  = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

















void USART_Init(USART_Handle_t *pUSARTHandle)
{

	uint32_t tempreg=0;
	//enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);
	//Configuration of CR1

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_RE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// enable both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//ode to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    // code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

// Configuration of CR2

	tempreg=0;

	//code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

//Configuration of CR3

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{

		//code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE);

	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

// Configuration of BRR

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}





void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TXE));

         //USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);


			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TC));
}




void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable TXE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable TC interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}



uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Enable RXNE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 * BaudRate));

  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));


  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);


   }else
   {
	   //over sampling by 16
	   F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);


   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR  = tempreg;
}




uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t PCLK1,SYSTEMCLK;
	uint8_t CLKSRC,temp,AHBP,APB1P;

	CLKSRC = (RCC->CFGR >> 2) & 0x3;

	if(CLKSRC == 0)
	{
		SYSTEMCLK = 16000000;
	}else if(CLKSRC == 1)
	{
		SYSTEMCLK = 8000000;
	}else if(CLKSRC == 2)
	{
		//SYSTEMCLK = RCC_GetPLLOutputClock();
	}


	//AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		AHBP = 1;
	}else
	{
		AHBP = AHB_PreScaler[temp-8];
	}
	//APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		APB1P = 1;
	}else
	{
		APB1P = APB1_PreScaler[temp-4];
	}

	PCLK1 = (SYSTEMCLK/AHBP)/APB1P;



	return PCLK1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t PCLK2,SYSTEMCLK;
	uint8_t CLKSRC,temp,AHBP,APB2P;

	CLKSRC = (RCC->CFGR >> 2) & 0x3;

	if(CLKSRC == 0)
	{
		SYSTEMCLK = 16000000;
	}
	else if(CLKSRC == 1)
	{
		SYSTEMCLK = 8000000;
	}
	else if(CLKSRC == 2)
	{
		//SYSTEMCLK = RCC_GetPLLOutputClock();
	}

	//AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		AHBP = 1;
	}
	else
	{
		AHBP = AHB_PreScaler[temp-8];
	}
	//APB2
	temp = ((RCC->CFGR >> 13) & 0x7);  // note: bit positions 15:13 for APB2

	if(temp < 4)
	{
		APB2P = 1;
	}
	else
	{
		APB2P = APB1_PreScaler[temp-4]; // same table for APB1 and APB2 prescaler
	}

	PCLK2 = (SYSTEMCLK/AHBP)/APB2P;

	return PCLK2;
}


void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

	//Check if TC flag is set
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	//Check if TC interrupt is enabled
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		//This interrupt is because of TC

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//If TxLen is zero, close the transmission
			if(!pUSARTHandle->TxLen)
			{
				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Disable TC interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset buffer pointer
				pUSARTHandle->pTxBuffer = NULL;

				//Reset length
				pUSARTHandle->TxLen = 0;

				//Notify application
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Check if TXE flag is set
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Check if TXE interrupt is enabled
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		//This interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending until TxLen becomes zero
			if(pUSARTHandle->TxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					uint16_t *pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity → increment buffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Decrement length by 2
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity enabled → increment once
						pUSARTHandle->pTxBuffer++;

						//Decrement length by 1
						pUSARTHandle->TxLen -= 1;
					}
				}
				else
				{
					//8-bit transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					//Increment buffer
					pUSARTHandle->pTxBuffer++;

					//Decrement length
					pUSARTHandle->TxLen--;
				}
			}

			if(pUSARTHandle->TxLen == 0)
			{
				//Disable TXE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		//This interrupt is because of RXNE

		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}

			if(!pUSARTHandle->RxLen)
			{
				//Disable RXNE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

/*************************Check for CTS flag ********************************************/

	//Check if CTS flag is set
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Check if CTS interrupt is enabled
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 &&temp3)
	{
		//Clear CTS flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//Notify application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

/*************************Check for IDLE flag ********************************************/

	//Check if IDLE flag is set
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Check if IDLE interrupt is enabled
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//Clear IDLE flag by reading SR and DR
		temp1 = pUSARTHandle->pUSARTx->SR;
		temp1 = pUSARTHandle->pUSARTx->DR;

		//Notify application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

/*************************Check for Overrun error ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1 && temp2)
	{
		//Notify application about overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}

/*************************Check for Error flags ********************************************/

	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;

		if(temp1 & (1 << USART_SR_FE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_FE);
		}

		if(temp1 & (1 << USART_SR_NF))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_NE);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_ORE);
		}
	}
}

