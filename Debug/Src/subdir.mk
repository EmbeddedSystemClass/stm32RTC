################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/fonts.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/rtc.c \
../Src/ssd1306.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/usart.c 

OBJS += \
./Src/fonts.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/rtc.o \
./Src/ssd1306.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/usart.o 

C_DEPS += \
./Src/fonts.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/rtc.d \
./Src/ssd1306.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F103xB '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/Inc" -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/Drivers/CMSIS/Include" -I"/home/medprime/Documents/SystemWorkBench/RTC/stm32RTC/u8glib/inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


