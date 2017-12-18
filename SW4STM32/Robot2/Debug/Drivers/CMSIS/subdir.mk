################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Andy/Documents/Robot2/Src/system_stm32f3xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f3xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f3xx.o: C:/Users/Andy/Documents/Robot2/Src/system_stm32f3xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/Andy/Documents/Robot2/Inc" -I"C:/Users/Andy/Documents/Robot2/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/Andy/Documents/Robot2/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Andy/Documents/Robot2/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/Andy/Documents/Robot2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


