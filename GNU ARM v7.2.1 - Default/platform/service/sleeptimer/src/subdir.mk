################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/service/sleeptimer/src/sl_sleeptimer.c \
../platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.c \
../platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.c 

OBJS += \
./platform/service/sleeptimer/src/sl_sleeptimer.o \
./platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o \
./platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o 

C_DEPS += \
./platform/service/sleeptimer/src/sl_sleeptimer.d \
./platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.d \
./platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.d 


# Each subdirectory must supply rules for building sources it contributes
platform/service/sleeptimer/src/sl_sleeptimer.o: ../platform/service/sleeptimer/src/sl_sleeptimer.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-D__STACK_SIZE=0x800' '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__StackLimit=0x20000000' '-DNVM3_DEFAULT_NVM_SIZE=24576' '-DEFR32MG22C224F512IM40=1' -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\drivers" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\soc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\halconfig" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\CMSIS\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\app\bluetooth\common\util" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\halconfig\inc\hal-config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\bsp" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\chip\efr32\efr32xg2x" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ble" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\uartdrv\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source\GCC" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader\api" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\EFR32MG22_BRD4182A\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader" -O3 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=hard -MMD -MP -MF"platform/service/sleeptimer/src/sl_sleeptimer.d" -MT"platform/service/sleeptimer/src/sl_sleeptimer.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o: ../platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-D__STACK_SIZE=0x800' '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__StackLimit=0x20000000' '-DNVM3_DEFAULT_NVM_SIZE=24576' '-DEFR32MG22C224F512IM40=1' -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\drivers" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\soc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\halconfig" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\CMSIS\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\app\bluetooth\common\util" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\halconfig\inc\hal-config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\bsp" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\chip\efr32\efr32xg2x" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ble" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\uartdrv\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source\GCC" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader\api" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\EFR32MG22_BRD4182A\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader" -O3 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=hard -MMD -MP -MF"platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.d" -MT"platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o: ../platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-D__STACK_SIZE=0x800' '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__StackLimit=0x20000000' '-DNVM3_DEFAULT_NVM_SIZE=24576' '-DEFR32MG22C224F512IM40=1' -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\drivers" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\protocol\bluetooth\ble_stack\inc\soc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\halconfig" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\CMSIS\Include" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\app\bluetooth\common\util" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emlib\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\nvm3\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\common" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\halconfig\inc\hal-config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\common\bsp" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\chip\efr32\efr32xg2x" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\common\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\service\sleeptimer\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\radio\rail_lib\protocol\ble" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\uartdrv\inc" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\emdrv\sleep\src" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source\GCC" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\Device\SiliconLabs\EFR32MG22\Source" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader\api" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\hardware\kit\EFR32MG22_BRD4182A\config" -I"C:\Users\kokhe\OneDrive\Documents\GitHub\smartdoorlock_efr32xg22\platform\bootloader" -O3 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=hard -MMD -MP -MF"platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.d" -MT"platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


