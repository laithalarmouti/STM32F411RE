#include "STM32F411RE.h"
#include "STM32F411RE_GPIO_DRIVER.h"




#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOBTN;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_LO;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYP_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PP;

		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(&GPIOLed);


	GPIOBTN.pGPIOx = GPIOC;
	GPIOBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FT;
	GPIOBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_MD;
	GPIOBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		GPIO_PeriClockControl(GPIOC, ENABLE);
		GPIO_Init(&GPIOBTN);

		GPIO_IRQPriorityConfig(IRQ_NO_EXTI10_15,15);
		GPIO_IRQInteruptConfig(IRQ_NO_EXTI10_15,ENABLE);


while(1);


}


void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
}




