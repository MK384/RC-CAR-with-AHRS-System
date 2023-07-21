################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Middlewares/MPU6050_MotionApp/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/Board_HAL_Drivers/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := RC_Car_Node
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RC_Car_Node.elf \

MAP_FILES += \
RC_Car_Node.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RC_Car_Node.list \

OBJCOPY_HEX += \
RC_Car_Node.hex \


# All Target
all: main-build

# Main-build Target
main-build: RC_Car_Node.elf secondary-outputs

# Tool invocations
RC_Car_Node.elf RC_Car_Node.map: $(OBJS) $(USER_OBJS) E:\RC-CAR-with-AHRS-System\Car_Node\STM32F401CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "RC_Car_Node.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m4 -T"E:\RC-CAR-with-AHRS-System\Car_Node\STM32F401CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RC_Car_Node.map" -Wl,--gc-sections -static -L../Middlewares/ST/ARM/DSP/Lib -L../Middlewares/ST/STM32_MotionAC_Library/Lib -L../Middlewares/ST/STM32_MotionAW_Library/Lib -L../Middlewares/ST/STM32_MotionEC_Library/Lib -L../Middlewares/ST/STM32_MotionFX_Library/Lib -L../Middlewares/ST/STM32_MotionGC_Library/Lib -L../Middlewares/ST/STM32_MotionID_Library/Lib -L../Middlewares/ST/STM32_MotionMC_Library/Lib -L../Middlewares/ST/STM32_MotionPM_Library/Lib -L../Middlewares/ST/STM32_MotionPW_Library/Lib -L../Middlewares/ST/STM32_MotionTL_Library/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RC_Car_Node.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RC_Car_Node.list"
	@echo 'Finished building: $@'
	@echo ' '

RC_Car_Node.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "RC_Car_Node.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) RC_Car_Node.elf RC_Car_Node.hex RC_Car_Node.list RC_Car_Node.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
