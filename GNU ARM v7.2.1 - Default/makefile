################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32MG22/GCC/subdir.mk
-include platform/service/sleeptimer/src/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/ustimer/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/emdrv/nvm3/src/subdir.mk
-include platform/emdrv/gpiointerrupt/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32MG22/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32MG22/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include app/bluetooth/common/util/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: smartdoorlock_efr32xg22.axf

# Tool invocations
smartdoorlock_efr32xg22.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\efr32mg22c224f512im40.ld" -Xlinker --gc-sections -Xlinker -Map="smartdoorlock_efr32xg22.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o smartdoorlock_efr32xg22.axf -Wl,--start-group "./app/bluetooth/common/util/infrastructure.o" "./application_properties.o" "./battery.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./main.o" "./pti.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/cpt212b.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/si7013.o" "./hardware/kit/common/drivers/tempsens.o" "./hardware/kit/common/drivers/udelay.o" "./platform/Device/SiliconLabs/EFR32MG22/Source/GCC/startup_efr32mg22.o" "./platform/Device/SiliconLabs/EFR32MG22/Source/system_efr32mg22.o" "./platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./platform/emdrv/nvm3/src/nvm3_default.o" "./platform/emdrv/nvm3/src/nvm3_hal_flash.o" "./platform/emdrv/nvm3/src/nvm3_lock.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emdrv/ustimer/src/ustimer.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_burtc.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_eusart.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_iadc.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./platform/service/sleeptimer/src/sl_sleeptimer.o" "./platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\protocol\bluetooth\lib\EFR32MG22\GCC\libbluetooth.a" "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\protocol\bluetooth\lib\EFR32MG22\GCC\libmbedtls.a" "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\protocol\bluetooth\lib\EFR32MG22\GCC\binapploader.o" "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\platform\emdrv\nvm3\lib\libnvm3_CM33_gcc.a" "C:\Users\kokhe\SimplicityStudio\v4_workspace\smartdoorlock_efr32xg22\platform\radio\rail_lib\autogen\librail_release\librail_efr32xg22_gcc_release.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: smartdoorlock_efr32xg22.hex'
	arm-none-eabi-objcopy -O ihex "smartdoorlock_efr32xg22.axf" "smartdoorlock_efr32xg22.hex"
	@echo ' '

	@echo 'Building bin file: smartdoorlock_efr32xg22.bin'
	arm-none-eabi-objcopy -O binary "smartdoorlock_efr32xg22.axf" "smartdoorlock_efr32xg22.bin"
	@echo ' '

	@echo 'Building s37 file: smartdoorlock_efr32xg22.s37'
	arm-none-eabi-objcopy -O srec "smartdoorlock_efr32xg22.axf" "smartdoorlock_efr32xg22.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "smartdoorlock_efr32xg22.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) smartdoorlock_efr32xg22.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
