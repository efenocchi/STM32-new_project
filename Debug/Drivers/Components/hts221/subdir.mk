################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/hts221/hts221.c 

OBJS += \
./Drivers/Components/hts221/hts221.o 

C_DEPS += \
./Drivers/Components/hts221/hts221.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/hts221/hts221.o: ../Drivers/Components/hts221/hts221.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/B-L475E-IOT01" -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/Components" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Components/hts221/hts221.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

