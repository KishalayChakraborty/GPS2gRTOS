################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := RTOS_GPSv1_DUMMYAPP_FOTA
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RTOS_GPSv1_DUMMYAPP_FOTA.elf \

MAP_FILES += \
RTOS_GPSv1_DUMMYAPP_FOTA.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RTOS_GPSv1_DUMMYAPP_FOTA.list \

OBJCOPY_HEX += \
RTOS_GPSv1_DUMMYAPP_FOTA.hex \

OBJCOPY_BIN += \
RTOS_GPSv1_DUMMYAPP_FOTA.bin \


# All Target
all: main-build

# Main-build Target
main-build: RTOS_GPSv1_DUMMYAPP_FOTA.elf secondary-outputs

# Tool invocations
RTOS_GPSv1_DUMMYAPP_FOTA.elf RTOS_GPSv1_DUMMYAPP_FOTA.map: $(OBJS) $(USER_OBJS) C:\Users\dell\Documents\GitHub\GPS2gRTOS\RTOS_GPS2g_V1_DUMMYAPP_FOTA\RTOS_GPS2g_V1_DUMMYAPP_FOTA\STM32G070CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "RTOS_GPSv1_DUMMYAPP_FOTA.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\dell\Documents\GitHub\GPS2gRTOS\RTOS_GPS2g_V1_DUMMYAPP_FOTA\RTOS_GPS2g_V1_DUMMYAPP_FOTA\STM32G070CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RTOS_GPSv1_DUMMYAPP_FOTA.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RTOS_GPSv1_DUMMYAPP_FOTA.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RTOS_GPSv1_DUMMYAPP_FOTA.list"
	@echo 'Finished building: $@'
	@echo ' '

RTOS_GPSv1_DUMMYAPP_FOTA.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "RTOS_GPSv1_DUMMYAPP_FOTA.hex"
	@echo 'Finished building: $@'
	@echo ' '

RTOS_GPSv1_DUMMYAPP_FOTA.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RTOS_GPSv1_DUMMYAPP_FOTA.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) RTOS_GPSv1_DUMMYAPP_FOTA.bin RTOS_GPSv1_DUMMYAPP_FOTA.elf RTOS_GPSv1_DUMMYAPP_FOTA.hex RTOS_GPSv1_DUMMYAPP_FOTA.list RTOS_GPSv1_DUMMYAPP_FOTA.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
