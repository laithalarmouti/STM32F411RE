
//THIS IS STM32F411RE_GPIO_DRIVER.C

#include "STM32F411RE_GPIO_DRIVER.h"

//CLOCK SETUP
/*********************************************************************
 * @Fn GPIO_PeriClockControl
 *
 * @Brief
 *
 * THIS FUNCTION ENABLES OR DISABLES PERIPHERAL CLOCK FOR A GIVEN GPIO PORT
 *
 * @PARAMS
 *  1- BASE ADDRESS OF GPIO PAREPHERAL
 *  2- ENABLE OR DISABLE MACRO
 *
 * @Return
 * NONE
 *
 *
 * @NOTES
 * NONE
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
	    }
	}else if(EnOrDi == DISABLE) {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
				{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
				{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
				{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOH)
				{
			GPIOH_PCLK_DI();
				}
	}

}



//ENABLE DISABLE
/*********************************************************************
 * @Fn  GPIO_Init
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	//clock enable
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);



	//1- CONFIGURE THE MODE OF THE PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //CHECK IF ITS INTERUPT MODE OR NOT
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //CLEAR
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT)
		{
			//configure FTSR register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR RTSR JUST INCASE BEST PRACTICE
			EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RT)
		{
			//configure RTSR register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR FTSR JUST INCASE BEST PRACTCE
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else
		{
			//configure both
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//-CONFIG THE GPIO PORT SELECTION IN SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; //Determines which CR Registers,
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;	// BIT allocation
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);




		//- ENABLE THE EXTI INTERUPT DELIVERY USING IMR INTERUPT MASK REGISTER
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2-CONFIGURE THE SPEED OF THE PIN
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3-CONFIGURE PULLUP PULLDOWN SETTINGS
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4- OUTPUT TYPE OF PIN
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5- CONFIGURE THE ALT FUNCTIONALITY
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4*temp2));

	}


}


/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


//READ AND WRITE
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber)&0x1);
}
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);
}
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->BSRR = (1<<PinNumber);   //SETS THE PIN TO 1  FOR BSR REGISTER THE FIRST 16 BITS ARE USED TO SET 0-15, AND 15-31 are used to RESET
	}else
	{
		pGPIOx->BSRR = (1<<(PinNumber+16));  //RESETS THE PIN to 0
	}
}
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->BSRR = (Value & 0xFFFF) | ((~Value & 0xFFFF) << 16); // GO BACK TO LECTURE 100
}
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


//IRQ CONFIGURATION AND ISR HANDLING
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDi )
{
	if(EnOrDi == ENABLE)
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
	else if (EnOrDi == DISABLE)
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


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//first step FIND THE IPR REGISTER
	uint8_t iprx = IRQNumber / 4;
	//which section of the register
	uint8_t iprx_section = IRQNumber % 4;

	//DUE to the architecture of this MCU, the 4 RIGHT BITS OF EACH BYTE is not utilized, so we have to shift accordingly
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount);
}
/*********************************************************************
 * @Fn
 *
 * @Brief
 *
 * @PARAMS
 *
 *
 * @Return
 *
 * @NOTES
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 <<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);
	}
}
