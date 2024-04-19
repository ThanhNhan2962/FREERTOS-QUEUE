################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FREERTOS/croutine.c \
../Drivers/FREERTOS/event_groups.c \
../Drivers/FREERTOS/list.c \
../Drivers/FREERTOS/queue.c \
../Drivers/FREERTOS/stream_buffer.c \
../Drivers/FREERTOS/tasks.c \
../Drivers/FREERTOS/timers.c 

OBJS += \
./Drivers/FREERTOS/croutine.o \
./Drivers/FREERTOS/event_groups.o \
./Drivers/FREERTOS/list.o \
./Drivers/FREERTOS/queue.o \
./Drivers/FREERTOS/stream_buffer.o \
./Drivers/FREERTOS/tasks.o \
./Drivers/FREERTOS/timers.o 

C_DEPS += \
./Drivers/FREERTOS/croutine.d \
./Drivers/FREERTOS/event_groups.d \
./Drivers/FREERTOS/list.d \
./Drivers/FREERTOS/queue.d \
./Drivers/FREERTOS/stream_buffer.d \
./Drivers/FREERTOS/tasks.d \
./Drivers/FREERTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FREERTOS/%.o: ../Drivers/FREERTOS/%.c Drivers/FREERTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

