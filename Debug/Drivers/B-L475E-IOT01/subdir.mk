################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/B-L475E-IOT01/stm32l475e_iot01.c \
../Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.c \
../Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.c \
../Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.c 

OBJS += \
./Drivers/B-L475E-IOT01/stm32l475e_iot01.o \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.o \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.o \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.o 

C_DEPS += \
./Drivers/B-L475E-IOT01/stm32l475e_iot01.d \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.d \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.d \
./Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/B-L475E-IOT01/stm32l475e_iot01.o: ../Drivers/B-L475E-IOT01/stm32l475e_iot01.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/B-L475E-IOT01" -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/Components" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/B-L475E-IOT01/stm32l475e_iot01.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.o: ../Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/B-L475E-IOT01" -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/Components" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/B-L475E-IOT01/stm32l475e_iot01_hsensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.o: ../Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/B-L475E-IOT01" -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/Components" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/B-L475E-IOT01/stm32l475e_iot01_psensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.o: ../Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/B-L475E-IOT01" -I"C:/Users/Emanuele/STM32CubeIDE/workspace_1.4.0/base/Drivers/Components" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/B-L475E-IOT01/stm32l475e_iot01_tsensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

