################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Bare_Metal_MPU_6050.c \
../Src/sysmem.c 

OBJS += \
./Src/Bare_Metal_MPU_6050.o \
./Src/sysmem.o 

C_DEPS += \
./Src/Bare_Metal_MPU_6050.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Bare_Metal_MPU_6050.o: ../Src/Bare_Metal_MPU_6050.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/Asus/STM32CubeIDE/workspace_test/stm32f446re_re/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/Bare_Metal_MPU_6050.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/Asus/STM32CubeIDE/workspace_test/stm32f446re_re/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

