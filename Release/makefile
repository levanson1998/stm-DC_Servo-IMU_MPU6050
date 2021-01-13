################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/lib/subdir.mk
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
stm_thesis.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm_thesis.list \

OBJCOPY_BIN += \
stm_thesis.bin \


# All Target
all: stm_thesis.elf secondary-outputs

# Tool invocations
stm_thesis.elf: $(OBJS) $(USER_OBJS) E:\LAPTOP\HOC_TAP\DC-LVTN\stm-DC_Servo-IMU_MPU6050-master\ (1).zip_expanded\stm-DC_Servo-IMU_MPU6050-master\STM32F411VETX_FLASH.ld
	arm-none-eabi-gcc -o "stm_thesis.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\LAPTOP\HOC_TAP\DC-LVTN\stm-DC_Servo-IMU_MPU6050-master (1).zip_expanded\stm-DC_Servo-IMU_MPU6050-master\STM32F411VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm_thesis.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm_thesis.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm_thesis.list"
	@echo 'Finished building: $@'
	@echo ' '

stm_thesis.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm_thesis.bin"
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
