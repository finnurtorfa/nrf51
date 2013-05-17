### Generated makefile ###

DEVICE := NRF51
BOARD := BOARD_NRF6310
DEVICESERIES := nrf51

C_SOURCE_FILES += ../main_host_ack_payload.c
C_SOURCE_FILES += ../../../../../Source/templates/system_nrf51.c 
LIBRARIES =  ../../../../../Lib/gzll/gcc/gzll_gcc.a

OUTPUT_FILENAME := gzll_host_ack_payload
LINKER_SCRIPT := ../../../../../Source/templates/gcc/gcc_linker_script_$(DEVICESERIES).ld
ASSEMBLER_SOURCE_FILES += ../../../../../Source/templates/gcc/gcc_startup_$(DEVICESERIES).s

INCLUDEPATHS += -I../../../../../Include
INCLUDEPATHS += -I../../../../../Include/gzll
INCLUDEPATHS += -I../../../../../Include/gcc

# Hardware definitions
CPU := cortex-m0

####################################################################
# Toolchain                                                        #
# You might need to change paths/etc to match your system          #
# Tools are assumed to be in the PATH                              #
####################################################################

GNU_INSTALL_ROOT := C:/Program Files (x86)/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI
GNU_VERSION := 4.6.3

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-gcc-$(GNU_VERSION)"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-objcopy"
READELF  		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-readelf"
CODESIZE 		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-size"
SIMULATOR 		:= "$(GNU_INSTALL_ROOT)/bin/arm-none-eabi-run"
MK 				:= mkdir
RM 				:= rd /S /Q

# Programmer
PROGRAMMER		:= nrfjprog.exe --reset --program 

# Build directories
OBJECT_DIRECTORY := _build
OUTPUT_BINARY_DIRECTORY := _build
LISTING_DIRECTORY := _build

# Linker flags
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/arm-none-eabi/lib/armv6-m"
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/lib/gcc/arm-none-eabi/$(GNU_VERSION)/armv6-m"
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -T$(LINKER_SCRIPT)

# Compiler flags
CFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -D$(DEVICE) -DDO_NOT_USE_DEPRECATED -D$(BOARD) --std=gnu99
CFLAGS += -Wall

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

####################################################################
# Rules                                                            #
####################################################################

C_SOURCE_FILENAMES = $(notdir $(C_SOURCE_FILES) )
ASSEMBLER_SOURCE_FILENAMES = $(notdir $(ASSEMBLER_SOURCE_FILES) )

# Make a list of source paths
C_SOURCE_PATHS = $(sort $(dir $(C_SOURCE_FILES) ) )
ASSEMBLER_SOURCE_PATHS = $(sort $(dir $(ASSEMBLER_SOURCE_FILES) ) )

C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILENAMES:.c=.o) )
ASSEMBLER_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASSEMBLER_SOURCE_FILENAMES:.s=.o) )

# Set source lookup paths
vpath %.c $(C_SOURCE_PATHS)
vpath %.s $(ASSEMBLER_SOURCE_PATHS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJECT_DIRECTORY)/, $(COBJS:.o=.d))

## Default build target
.PHONY: all
all: release

## Remove build directories, including files inside them
.PHONY: clean
clean:
	- $(RM) $(BUILD_DIRECTORIES)

### Targets
.PHONY: debug
debug:    CFLAGS += -DDEBUG -g3 -O0
debug:    $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

.PHONY: release
release:  CFLAGS += -DNDEBUG -O3
release:  $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echostuff:
	echo $(C_OBJECTS)
	echo $(C_SOURCE_FILES)

## Create build directories
$(BUILD_DIRECTORIES):
	$(MK) $@

## Create objects from C source files
$(OBJECT_DIRECTORY)/%.o: %.c
# Build header dependencies
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -M $< -MF "$(@:.o=.d)" -MT $@
# Do the actual compilation
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Assemble .s files
$(OBJECT_DIRECTORY)/%.o: %.s
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Link C and assembler objects to an .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBRARIES)
	$(CC) $(LDFLAGS) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBRARIES) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

## Program device
.PHONY: flash
flash: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(PROGRAMMER) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
