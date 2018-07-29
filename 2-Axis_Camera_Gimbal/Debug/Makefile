################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../2-Axis_Camera_Gimbal.c \
../ADC.c \
../ak8975c.c \
../BDLC.c \
../eeploadsave.c \
../mpu9150.c \
../PID.c \
../setup.c \
../twi.c \
../txSerial.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
2-Axis_Camera_Gimbal.o \
ADC.o \
ak8975c.o \
BDLC.o \
eeploadsave.o \
mpu9150.o \
PID.o \
setup.o \
twi.o \
txSerial.o

OBJS_AS_ARGS +=  \
2-Axis_Camera_Gimbal.o \
ADC.o \
ak8975c.o \
BDLC.o \
eeploadsave.o \
mpu9150.o \
PID.o \
setup.o \
twi.o \
txSerial.o

C_DEPS +=  \
2-Axis_Camera_Gimbal.d \
ADC.d \
ak8975c.d \
BDLC.d \
eeploadsave.d \
mpu9150.d \
PID.d \
setup.d \
twi.d \
txSerial.d

C_DEPS_AS_ARGS +=  \
2-Axis_Camera_Gimbal.d \
ADC.d \
ak8975c.d \
BDLC.d \
eeploadsave.d \
mpu9150.d \
PID.d \
setup.d \
twi.d \
txSerial.d

OUTPUT_FILE_PATH +=2-Axis_Camera_Gimbal.elf

OUTPUT_FILE_PATH_AS_ARGS +=2-Axis_Camera_Gimbal.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler





















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega1284 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="2-Axis_Camera_Gimbal.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mrelax -mmcu=atmega1284  
	@echo Finished building target: $@
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "2-Axis_Camera_Gimbal.elf" "2-Axis_Camera_Gimbal.hex"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "2-Axis_Camera_Gimbal.elf" "2-Axis_Camera_Gimbal.eep" || exit 0
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "2-Axis_Camera_Gimbal.elf" > "2-Axis_Camera_Gimbal.lss"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "2-Axis_Camera_Gimbal.elf" "2-Axis_Camera_Gimbal.srec"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-size.exe" "2-Axis_Camera_Gimbal.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "2-Axis_Camera_Gimbal.elf" "2-Axis_Camera_Gimbal.a" "2-Axis_Camera_Gimbal.hex" "2-Axis_Camera_Gimbal.lss" "2-Axis_Camera_Gimbal.eep" "2-Axis_Camera_Gimbal.map" "2-Axis_Camera_Gimbal.srec" "2-Axis_Camera_Gimbal.usersignatures"
	