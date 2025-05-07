#include <string.h>
#include "STM32F411RE.h"
#include "STM32F411RE_GPIO_DRIVER.h"
#include "STM32F411RE_SPI_DRIVER.h"


void delay(void){
	for(uint32_t i = 0; i<150000;i++);
}



//SPI2_MOSI PB15 AF05
//SPI2 MISO PB14 AF05
//SPI2 SCK PB13 AF05
//SPI2 PB12 NSS
//AF5

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYP_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_LO;
	//CLOCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}





void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SCLK_DIV128;
	SPI2Handle.SPI_Config.SPI_DFF = DATA_SIZE_8;
	SPI2Handle.SPI_Config.SPI_CPOL = CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin
	SPI_Init(&SPI2Handle);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBTN;
	GPIOBTN.pGPIOx = GPIOC;
	GPIOBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_MD;
	GPIOBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOC, ENABLE);
		GPIO_Init(&GPIOBTN);
}


int main (void)
{
	char user_data[] = "Hello world";
	GPIO_ButtonInit();
	//function used to initialize the gpio pins to behave as spi2
	SPI2_GPIOInits();
	//INTIALIZE SPI
	SPI2_Inits();

	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 1);
		//MAKES NSS SIGNAL INTERNALLY HIGH AND AVOID MODF ERR
		//SPI_SSIConfig(SPI2, ENABLE);
		delay();
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send Length of data first,
		uint8_t DataLen = strlen(user_data);
		SPI_SendData(SPI2,&DataLen,1);

		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}

