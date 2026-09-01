################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PeripheralDrivers/Src/encoders.c \
../PeripheralDrivers/Src/motors.c \
../PeripheralDrivers/Src/odom.c \
../PeripheralDrivers/Src/oled.c \
../PeripheralDrivers/Src/pid.c 

OBJS += \
./PeripheralDrivers/Src/encoders.o \
./PeripheralDrivers/Src/motors.o \
./PeripheralDrivers/Src/odom.o \
./PeripheralDrivers/Src/oled.o \
./PeripheralDrivers/Src/pid.o 

C_DEPS += \
./PeripheralDrivers/Src/encoders.d \
./PeripheralDrivers/Src/motors.d \
./PeripheralDrivers/Src/odom.d \
./PeripheralDrivers/Src/oled.d \
./PeripheralDrivers/Src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
PeripheralDrivers/Src/%.o PeripheralDrivers/Src/%.su: ../PeripheralDrivers/Src/%.c PeripheralDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Dell/Documents/CEG/Y3S1/MDPX/STM32X/PeripheralDrivers/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PeripheralDrivers-2f-Src

clean-PeripheralDrivers-2f-Src:
	-$(RM) ./PeripheralDrivers/Src/encoders.d ./PeripheralDrivers/Src/encoders.o ./PeripheralDrivers/Src/encoders.su ./PeripheralDrivers/Src/motors.d ./PeripheralDrivers/Src/motors.o ./PeripheralDrivers/Src/motors.su ./PeripheralDrivers/Src/odom.d ./PeripheralDrivers/Src/odom.o ./PeripheralDrivers/Src/odom.su ./PeripheralDrivers/Src/oled.d ./PeripheralDrivers/Src/oled.o ./PeripheralDrivers/Src/oled.su ./PeripheralDrivers/Src/pid.d ./PeripheralDrivers/Src/pid.o ./PeripheralDrivers/Src/pid.su

.PHONY: clean-PeripheralDrivers-2f-Src

