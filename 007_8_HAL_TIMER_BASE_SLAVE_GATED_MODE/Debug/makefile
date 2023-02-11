################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

BUILD_ARTIFACT_NAME := 007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.list \

OBJCOPY_BIN += \
007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.bin \


# All Target
all: main-build

# Main-build Target
main-build: 007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.elf secondary-outputs

# Tool invocations
007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.elf: $(OBJS) $(USER_OBJS) D:\STM32CubeIDE\workspace_1.5.1\007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\STM32CubeIDE\workspace_1.5.1\007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.list"
	@echo 'Finished building: $@'
	@echo ' '

007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "007_8_HAL_TIMER_BASE_SLAVE_GATED_MODE.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
