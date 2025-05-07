#include "STM32F411RE.h"
#include "STM32F411RE_GPIO_DRIVER.h"


int main(void)
{
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
	    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Toggle PA5 (LED ON/OFF)
	    for (volatile int i = 0; i < 150000; i++); // Adjusted delay for ~3secs
	}

	return 0;
}
