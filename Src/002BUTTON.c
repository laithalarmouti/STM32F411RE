#include "STM32F411RE.h"
#include "STM32F411RE_GPIO_DRIVER.h"



void delay(void){
	for(uint32_t i = 0; i<150000;i++);
}
int main(void)
{
	GPIO_Handle_t GPIOBTN;
	GPIOBTN.pGPIOx = GPIOC;
	GPIOBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_MD;
	GPIOBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

		GPIO_PeriClockControl(GPIOC, ENABLE);
		GPIO_Init(&GPIOBTN);






	//PC13

	GPIO_Handle_t GPIOLED;
	GPIOLED.pGPIOx = GPIOA;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYP_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_MD;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOLED);

	while (1)
	{

		if (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) {
		    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET); // Turn on LED
		} else {
		    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET); // Turn off LED
		}
	}

	return 0;
}
