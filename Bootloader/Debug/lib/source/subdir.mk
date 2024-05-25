################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/source/aes_decrypt.c \
../lib/source/aes_encrypt.c \
../lib/source/cbc_mode.c \
../lib/source/ccm_mode.c \
../lib/source/cmac_mode.c \
../lib/source/ctr_mode.c \
../lib/source/ctr_prng.c \
../lib/source/ecc.c \
../lib/source/ecc_dh.c \
../lib/source/ecc_dsa.c \
../lib/source/ecc_platform_specific.c \
../lib/source/hmac.c \
../lib/source/hmac_prng.c \
../lib/source/sha256.c \
../lib/source/utils.c 

OBJS += \
./lib/source/aes_decrypt.o \
./lib/source/aes_encrypt.o \
./lib/source/cbc_mode.o \
./lib/source/ccm_mode.o \
./lib/source/cmac_mode.o \
./lib/source/ctr_mode.o \
./lib/source/ctr_prng.o \
./lib/source/ecc.o \
./lib/source/ecc_dh.o \
./lib/source/ecc_dsa.o \
./lib/source/ecc_platform_specific.o \
./lib/source/hmac.o \
./lib/source/hmac_prng.o \
./lib/source/sha256.o \
./lib/source/utils.o 

C_DEPS += \
./lib/source/aes_decrypt.d \
./lib/source/aes_encrypt.d \
./lib/source/cbc_mode.d \
./lib/source/ccm_mode.d \
./lib/source/cmac_mode.d \
./lib/source/ctr_mode.d \
./lib/source/ctr_prng.d \
./lib/source/ecc.d \
./lib/source/ecc_dh.d \
./lib/source/ecc_dsa.d \
./lib/source/ecc_platform_specific.d \
./lib/source/hmac.d \
./lib/source/hmac_prng.d \
./lib/source/sha256.d \
./lib/source/utils.d 


# Each subdirectory must supply rules for building sources it contributes
lib/source/%.o lib/source/%.su lib/source/%.cyclo: ../lib/source/%.c lib/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/X1/Downloads/Compressed/STM32F103C8T6-Bootloader-master/STM32F103C8T6-Bootloader-master/Bootloader/lib/source" -I"C:/Users/X1/Downloads/Compressed/STM32F103C8T6-Bootloader-master/STM32F103C8T6-Bootloader-master/Bootloader/lib/include/tinycrypt" -I../lib/include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib-2f-source

clean-lib-2f-source:
	-$(RM) ./lib/source/aes_decrypt.cyclo ./lib/source/aes_decrypt.d ./lib/source/aes_decrypt.o ./lib/source/aes_decrypt.su ./lib/source/aes_encrypt.cyclo ./lib/source/aes_encrypt.d ./lib/source/aes_encrypt.o ./lib/source/aes_encrypt.su ./lib/source/cbc_mode.cyclo ./lib/source/cbc_mode.d ./lib/source/cbc_mode.o ./lib/source/cbc_mode.su ./lib/source/ccm_mode.cyclo ./lib/source/ccm_mode.d ./lib/source/ccm_mode.o ./lib/source/ccm_mode.su ./lib/source/cmac_mode.cyclo ./lib/source/cmac_mode.d ./lib/source/cmac_mode.o ./lib/source/cmac_mode.su ./lib/source/ctr_mode.cyclo ./lib/source/ctr_mode.d ./lib/source/ctr_mode.o ./lib/source/ctr_mode.su ./lib/source/ctr_prng.cyclo ./lib/source/ctr_prng.d ./lib/source/ctr_prng.o ./lib/source/ctr_prng.su ./lib/source/ecc.cyclo ./lib/source/ecc.d ./lib/source/ecc.o ./lib/source/ecc.su ./lib/source/ecc_dh.cyclo ./lib/source/ecc_dh.d ./lib/source/ecc_dh.o ./lib/source/ecc_dh.su ./lib/source/ecc_dsa.cyclo ./lib/source/ecc_dsa.d ./lib/source/ecc_dsa.o ./lib/source/ecc_dsa.su ./lib/source/ecc_platform_specific.cyclo ./lib/source/ecc_platform_specific.d ./lib/source/ecc_platform_specific.o ./lib/source/ecc_platform_specific.su ./lib/source/hmac.cyclo ./lib/source/hmac.d ./lib/source/hmac.o ./lib/source/hmac.su ./lib/source/hmac_prng.cyclo ./lib/source/hmac_prng.d ./lib/source/hmac_prng.o ./lib/source/hmac_prng.su ./lib/source/sha256.cyclo ./lib/source/sha256.d ./lib/source/sha256.o ./lib/source/sha256.su ./lib/source/utils.cyclo ./lib/source/utils.d ./lib/source/utils.o ./lib/source/utils.su

.PHONY: clean-lib-2f-source

