################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32F411RE_GPIO_DRIVER.c \
../Drivers/Src/STM32F411RE_I2C_DRIVER.c \
../Drivers/Src/STM32F411RE_SPI_DRIVER.c \
../Drivers/Src/STM32F411RE_USART_DRIVER.c 

OBJS += \
./Drivers/Src/STM32F411RE_GPIO_DRIVER.o \
./Drivers/Src/STM32F411RE_I2C_DRIVER.o \
./Drivers/Src/STM32F411RE_SPI_DRIVER.o \
./Drivers/Src/STM32F411RE_USART_DRIVER.o 

C_DEPS += \
./Drivers/Src/STM32F411RE_GPIO_DRIVER.d \
./Drivers/Src/STM32F411RE_I2C_DRIVER.d \
./Drivers/Src/STM32F411RE_SPI_DRIVER.d \
./Drivers/Src/STM32F411RE_USART_DRIVER.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"C:/Users/MSI/Desktop/MCU/MCU1/STM32F411RE/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32F411RE_GPIO_DRIVER.cyclo ./Drivers/Src/STM32F411RE_GPIO_DRIVER.d ./Drivers/Src/STM32F411RE_GPIO_DRIVER.o ./Drivers/Src/STM32F411RE_GPIO_DRIVER.su ./Drivers/Src/STM32F411RE_I2C_DRIVER.cyclo ./Drivers/Src/STM32F411RE_I2C_DRIVER.d ./Drivers/Src/STM32F411RE_I2C_DRIVER.o ./Drivers/Src/STM32F411RE_I2C_DRIVER.su ./Drivers/Src/STM32F411RE_SPI_DRIVER.cyclo ./Drivers/Src/STM32F411RE_SPI_DRIVER.d ./Drivers/Src/STM32F411RE_SPI_DRIVER.o ./Drivers/Src/STM32F411RE_SPI_DRIVER.su ./Drivers/Src/STM32F411RE_USART_DRIVER.cyclo ./Drivers/Src/STM32F411RE_USART_DRIVER.d ./Drivers/Src/STM32F411RE_USART_DRIVER.o ./Drivers/Src/STM32F411RE_USART_DRIVER.su

.PHONY: clean-Drivers-2f-Src

