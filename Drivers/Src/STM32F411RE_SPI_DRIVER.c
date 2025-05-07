
#include "STM32F411RE_SPI_DRIVER.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}else if(pSPIx == SPI5)
			{
				SPI5_PCLK_EN();
			}
		}else if(EnOrDi == DISABLE) {
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
					{
				SPI3_PCLK_DI();
			}else if(pSPIx == SPI4)
					{
				SPI4_PCLK_DI();
			}else if(pSPIx == SPI5)
					{
				SPI5_PCLK_DI();
			}
		}



}


//INIT AND DEINIT
void SPI_Init(SPI_Handle_t *pSPIHandle){


	uint32_t tempreg = 0;


	//CLOCK enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

//DEVICE MODE
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR ;
	//-----------------------
//BUS  MODE CONFIG
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//CLEAR BIDIMODE
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//ENABLE BIDIMODE
		tempreg |= 1<<SPI_CR1_BIDIMODE;
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//CLEARBIDI MODE AND RXONLY BIT MUST BE SET
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		tempreg |= 1<<SPI_CR1_RXONLY;
	}
//SPI_SclkSpeed
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed <<SPI_CR1_BR  ;
//DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF<< SPI_CR1_DFF;
//CPOL
	tempreg|= pSPIHandle->SPI_Config.SPI_CPOL<<SPI_CR1_CPOL;
//CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA<<SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}




void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}else if(pSPIx == SPI4)
		{
			SPI4_REG_RESET();
		}else if(pSPIx == SPI5)
		{
			SPI5_REG_RESET();
		}
}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//DATA SEND AND RECEIVE

// -------------------BLOCKING TYPE CALL

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1.Wait until TXE is SET
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET)
			;
		//2. CHECK THE DFF BIT IN CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 BITS
			{
				pSPIx->DR = *((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}
			else  // 8 BITS
			{
				pSPIx->DR = *(pTxBuffer);
				Len--;
				pTxBuffer++;
			}

	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	//WAIT until RXNE IS SET
	while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)== FLAG_RESET)
		;
	if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//load the data from DR to RXBUFFER
		*((uint16_t*)pRxBuffer) = pSPIx->DR;
		Len--;
		Len--;
		(uint16_t*)pRxBuffer++;

	}
	else
	{
		*((uint8_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1<<SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
		}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}


// IRQ CONFIG AND ISR HANDLING

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//first step FIND THE IPR REGISTER
	uint8_t iprx = IRQNumber / 4;
	//which section of the register
	uint8_t iprx_section = IRQNumber % 4;

	//DUE to the architecture of this MCU, the 4 RIGHT BITS OF EACH BYTE is not utilized, so we have to shift accordingly
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	//we need to know what causes the interrupt,
	uint8_t temp1, temp2;
	//CHECKING FOR TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	//if both are true it means TXE triggered and it is ready to send DATA
	if(temp1 && temp2 )
	{
		//HANDLE TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//CHECK FOR RXNE
	temp1 = pHandle->pSPIx->SR & (1<< SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//HANDLE RXNE
		spi_rxne_interrupt_handle(pHandle);
	}
	//check for OVR overflow
	temp1 =pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 =pHandle->pSPIx->CR2 &(1<<SPI_CR2_ERRIE);
	if(temp1&&temp2)
	{
		//HANDLE OVRERR
		spi_ovr_err_interrupt_handle(pHandle);
	}

}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;


	if(state != SPI_BUSY_IN_TX)
	{
	//store tx buffer addres and len info
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	//mark SPI AS BUSY
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	//Set TXEIE control bit to get an interrupt when the TXE flag is set, rdy to send
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}

	return state;
}









uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pHandle->RxState;


	if(state != SPI_BUSY_IN_RX)
	{
	//store tx buffer addres and len info
	pHandle->pRxBuffer = pRxBuffer;
	pHandle->RxLen = Len;
	//mark SPI AS BUSY
	pHandle->TxState = SPI_BUSY_IN_RX;
	//Set TXEIE control bit to get an interrupt when the TXE flag is set, rdy to send
	pHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}

	return state;
}



//HELPER FUNCTIONS
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 BITS
				{
					pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
					pSPIHandle->TxLen--;
					pSPIHandle->TxLen--;
					(uint16_t*)pSPIHandle->pTxBuffer++;
				}
				else  // 8 BITS
				{
					pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
					pSPIHandle->TxLen--;
					pSPIHandle->pTxBuffer++;
				}
	if(!pSPIHandle->TxLen)
	{
		//stop communication txbuffer empty
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		 pSPIHandle->RxLen -= 2;
		 pSPIHandle->pRxBuffer--;
		 pSPIHandle->pRxBuffer--;
	}else
	{
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}
	if(!pSPIHandle->RxLen)
	{
		//reception complete turn off interupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}


}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
			uint8_t temp;
			//CLEAR OVR FLAG and INFORM APP
			if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
			{
				temp= pSPIHandle->pSPIx->DR;
				temp= pSPIHandle->pSPIx->SR;
			}
			(void)temp;
	//INFORM THE APP
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}



void SPI_CloseTransmission(SPI_Handle_t *pHandle)
{
			pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
			//reset buffers
			pHandle->pTxBuffer = NULL;
			pHandle->TxLen = 0;
			pHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pHandle)
{
			pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
			pHandle->pRxBuffer = NULL;
			pHandle->RxLen = 0;
			pHandle->RxState = SPI_READY;

}




void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
		uint8_t temp;
		temp= pSPIx->DR;
		temp= pSPIx->SR;
		(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}
