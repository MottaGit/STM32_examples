################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LM235_MEASURE_1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LM235_MEASURE_1.list \

OBJCOPY_BIN += \
LM235_MEASURE_1.bin \


# All Target
all: LM235_MEASURE_1.elf secondary-outputs

# Tool invocations
LM235_MEASURE_1.elf: $(OBJS) $(USER_OBJS) C:\Users\gabri\STM32CubeIDE\workspace_1.2.0\LM235_MEASURE_1\STM32G431KBTX_FLASH.ld
	arm-none-eabi-gcc -o "LM235_MEASURE_1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\gabri\STM32CubeIDE\workspace_1.2.0\LM235_MEASURE_1\STM32G431KBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LM235_MEASURE_1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LM235_MEASURE_1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LM235_MEASURE_1.list"
	@echo 'Finished building: $@'
	@echo ' '

LM235_MEASURE_1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LM235_MEASURE_1.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
