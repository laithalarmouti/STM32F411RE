
//THIS IS STM32F411RE_GPIO_DRIVER.H
#ifndef INC_STM32F411RE_GPIO_DRIVER_H_
#define INC_STM32F411RE_GPIO_DRIVER_H_

#include "STM32F411RE.h"




typedef struct {
	uint8_t GPIO_PinNumber;		 //@POSSIBLE VALUES FROM GPIO_PINNUMBERS
	uint8_t GPIO_PinMode;        //@ possible values from GPIO_PINMODES
	uint8_t GPIO_PinSpeed;		// @ POSSIBLE VALUES FROM GPIO_PINSPEED
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;  //this holds the base address of the gpio port to which the pin belongs.
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;




// GPIO_PINMODES
#define GPIO_MODE_IN 		0 	//INPUTMODE
#define GPIO_MODE_OUT 		1	//OUTPUTMODE
#define GPIO_MODE_ALTFN 	2	//ALTERNATING MODE
#define GPIO_MODE_ANALOG 	3	//ANALOG MODE
#define GPIO_MODE_IN_FT		4   //INPUT FALLING EDGE
#define GPIO_MODE_IN_RT		5   //INPUT RISING EDGE
#define GPIO_MODE_IN_RFT	6	//RISING/FALLING EDGE TRIGGER

//TYPES
#define GPIO_OP_TYP_PP		0   //PUSHPULL TYPE
#define	GPIO_OP_TYP_OD		1	//OPENDRAIN

//GPIO_PINSPEED
#define GPIO_SPD_LO			0	//LOW SPEED
#define GPIO_SPD_MD			1	//MEDIUM SPEED
#define GPIO_SPD_FT			2	//FAST SPEED
#define GPIO_SPD_HI			3	//HIGH SPEED

//PULL UP PULL DOWN REGI PP
#define GPIO_NO_PP			0 //NO PULL UP OR PULL DOWN
#define GPIO_PIN_PU			1 //PULL UP MODE
#define GPIO_PIN_PD			2 //PULLDOWN

//GPIO_PINNUMBERS
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


//*********************************************************************************************
//							API SUPPORTED BY THIS DRIVER
//				FOR MORE INFO CHECK FUNCTION DEFININTIONS
//**********************************************************************************************

//CLOCK SETUP
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


//ENABLE DISABLE
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//READ AND WRITE
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);


//IRQ CONFIGURATION AND ISR HANDLING
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDi );
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





















#endif
