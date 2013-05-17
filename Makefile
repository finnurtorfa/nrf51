#### Toolchain commands
####
GCC_INSTALL_ROOT	:= /opt/arm-2012.09
GCC_VERSION			 	:= 4.7.2
GCC_PREFIX			 	:= arm-none-eabi

CC      := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gcc"
AS      := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-as"
AR      := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ar" -r
LD      := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ld"
NM      := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-nm"
OBJDUMP := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objdump"
OBJCOPY := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objcopy"
GDB     := "$(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gdb"
CGDB    := "/usr/bin/cgdb"

MK	:= mkdir -p
RM	:= rm -rf

### General Variables
OUTPUT_NAME	= main
OUTPUT_DIR	= build
BIN_DIR			= bin
OBJ_DIR			= obj

### SDK Related variables
###
SDK 						 = lib/nrf51sdk/Nordic/nrf51822/
SDK_INCLUDE			 = $(SDK)Include/
SDK_SRC			 		 = $(SDK)Source/
SDK_TEMPLATE		 = $(SDK_SRC)templates/gcc/
SDK_APP_COMMON 	 = $(SDK_SRC)app_common/
SDK_EXT_SENSORS	 = $(SDK_SRC)ext_sensors/
SDK_BLE					 = $(SDK_SRC)ble/
SDK_BLE					+= $(SDK_SRC)ble/ble_services/

### Device related stuff
###
BOARD					:= BOARD_PCA100001
CPU						:= cortex-m0
DEVICE 				:= NRF51
DEVICESERIES 	:= nrf51

### Programmer
###
JLINK_DIR 			= ~/bin/jlink/
JLINK 					= $(JLINK_DIR)JLinkExe
JLINKGDBSERVER	= $(JLINK_DIR)JLinkGDBServer

FLASH_START_ADDRESS = 0x14000
SOFTDEVICE = lib/softdevice/s110_nrf51822_5.1.0_softdevice.hex

# Include directories
INCLUDEDIRS	 = include
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/include/
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/include-fixed/
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/$(GCC_PREFIX)/include/
INCLUDEDIRS	+= $(shell find $(SDK_INCLUDE) -type d)
 
### Source files
###

# Project Source
C_SRC  = main.c
C_SRC += battery.c
C_SRC += led.c

# APP Common
C_SRC += app_button.c
C_SRC += app_fifo.c
C_SRC += app_gpiote.c
C_SRC += app_scheduler.c
C_SRC += app_timer.c
C_SRC += app_uart.c

# BLE Services
C_SRC += ble_dis.c
C_SRC += ble_bas.c
C_SRC += ble_hrs.c
C_SRC += ble_hts.c

# BLE Libraries
C_SRC += ble_conn_params.c
C_SRC += ble_advdata.c
C_SRC += ble_srv_common.c
C_SRC += ble_stack_handler.c
C_SRC += ble_bondmngr.c
C_SRC += ble_flash.c
C_SRC += ble_radio_notification.c

# System Source
C_SRC += system_$(DEVICESERIES).c

### Assembly source files
ASSEMBLY_SRC = gcc_startup_$(DEVICESERIES).s

### Compiler related stuff
###
CFLAGS	 = -mcpu=$(CPU)
CFLAGS	+= -mthumb
CFLAGS	+= -mabi=aapcs
CFLAGS	+= --std=gnu99
CFLAGS	+= -Wall
CFLAGS	+= -D$(DEVICE)
CFLAGS	+= -D$(BOARD)
CFLAGS	+= $(patsubst %,-I%, $(INCLUDEDIRS))

### Linker related stuff
###
LDDIRS 	 = "$(GCC_INSTALL_ROOT)/$(GCC_PREFIX)/lib/armv6-m"
LDDIRS 	+= "$(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/armv6-m"
LDDIRS	+= $(SDK_TEMPLATE)

LD_SCRIPT = $(SDK_TEMPLATE)gcc_nrf51_s110_xxaa.ld

LDFLAGS  = -Xlinker 
LDFLAGS += -Map=$(BIN_DIR)/$(OUTPUT_NAME).map
LDFLAGS += -mcpu=$(CPU) 
LDFLAGS += -mthumb 
LDFLAGS += -mabi=aapcs 
LDFLAGS += -T$(LD_SCRIPT)
LDFLAGS	+= $(patsubst %,-L%, $(LDDIRS))

# Sorting removes duplicates
BUILD_DIRS := $(sort $(OBJ_DIR) $(OUTPUT_DIR) $(BIN_DIR) )

# Make a list of source paths
C_SRC_DIRS = src $(shell find $(SDK_SRC) -type d)
ASSEMBLY_SRC_DIRS = src $(shell find $(SDK_SRC) -type d)

# Object files
C_OBJ 				= $(addprefix $(OBJ_DIR)/, $(C_SRC:.c=.o))
ASSEMBLY_OBJ 	= $(addprefix $(OBJ_DIR)/, $(ASSEMBLY_SRC:.s=.o))

# Set source lookup paths
vpath %.c $(C_SRC_DIRS)
vpath %.s $(ASSEMBLY_SRC_DIRS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJ_DIR)/, $(C_OBJ:.o=.d))

### Rules
###
# Default build target
.PHONY : all
all : release

clean : 
	$(RM) $(OUTPUT_DIR)/*
	$(RM) $(OBJ_DIR)/*
	$(RM) $(BIN_DIR)/*
	- $(RM) JLink.log
	- $(RM) .gdbinit

.PHONY: release
release :  CFLAGS += -DNDEBUG -O3
release :  $(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(OUTPUT_DIR)/$(OUTPUT_NAME).hex

$(BUILD_DIRS) : 
	@echo 
	@echo "Creating directories"
	- $(MK) $@

# Create objects from C source files
$(OBJ_DIR)/%.o : %.c
	@echo
	@echo "Build header dependencies for file: " $<
	$(CC) $(CFLAGS) -M $< -MF "$(@:.o=.d)" -MT $@
	@echo
	@echo "Compiling: " $<
	$(CC) $(CFLAGS) -c -o $@ $<

## Assemble .s files
$(OBJ_DIR)/%.o : %.s
	@echo
	@echo "Compiling: " $<
	$(CC) $(patsubst %,-I%, $(INCLUDEDIRS)) -c -o $@ $<


## Link C and assembler objects to an .out file
$(OUTPUT_DIR)/$(OUTPUT_NAME).out : $(BUILD_DIRS) $(C_OBJ) $(ASSEMBLY_OBJ)
	@echo
	@echo "Linking object files: " 
	$(CC) $(LDFLAGS) $(C_OBJ) $(ASSEMBLY_OBJ) -o $(OUTPUT_DIR)/$(OUTPUT_NAME).out

## Create binary .bin file from the .out file
$(OUTPUT_DIR)/$(OUTPUT_NAME).bin : $(OUTPUT_DIR)/$(OUTPUT_NAME).out
	@echo
	@echo "Create binary(.bin) file from: " $<
	$(OBJCOPY) -O binary $(OUTPUT_DIR)/$(OUTPUT_NAME).out $(OUTPUT_DIR)/$(OUTPUT_NAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_DIR)/$(OUTPUT_NAME).hex : $(OUTPUT_DIR)/$(OUTPUT_NAME).out
	@echo
	@echo "Create binary(.hex) file from: " $<
	$(OBJCOPY) -O ihex $(OUTPUT_DIR)/$(OUTPUT_NAME).out $(OUTPUT_DIR)/$(OUTPUT_NAME).hex

## Program device
upload: rm.jlink upload.jlink stopdebug
	$(JLINK) $(OUTPUT_DIR)/upload.jlink

rm.jlink:
	-rm -rf $(OUTPUT_DIR)/upload.jlink
		
upload.jlink:
	echo "device nrf51822\nspeed 1000\nr\nloadbin $(OUTPUT_DIR)/$(OUTPUT_NAME).bin, $(FLASH_START_ADDRESS)\nr\ng\nexit\n" > $(OUTPUT_DIR)/upload.jlink
		  
upload-softdevice: upload-softdevice.jlink stopdebug
	@echo
	@echo "Convert from hex to binary. Split original hex in two to avoid huge (>250 MB) binary file with just 0s. "
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec3 $(SOFTDEVICE) $(OUTPUT_DIR)/_mainpart.bin
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec1 --remove-section .sec2 $(SOFTDEVICE) $(OUTPUT_DIR)/_uicr.bin
	$(JLINK) $(OUTPUT_DIR)/upload-softdevice.jlink

upload-softdevice.jlink:
	@echo
	@echo "Do magic. Write to NVMC to enable erase, do erase all and erase UICR, reset, enable writing, load mainpart bin, load uicr bin. Reset."
	@echo " Resetting in between is needed to disable the protections. "
	echo "w4 4001e504 1\nloadbin \"$(OUTPUT_DIR)/_mainpart.bin\" 0\nloadbin \"$(OUTPUT_DIR)/_uicr.bin\" 0x10001000\nr\ng\nexit\n" > $(OUTPUT_DIR)/upload-softdevice.jlink

recover: recover.jlink erase-all.jlink pin-reset.jlink
	$(JLINK) $(OUTPUT_DIR)/recover.jlink
	$(JLINK) $(OUTPUT_DIR)/erase-all.jlink
	$(JLINK) $(OUTPUT_DIR)/pin-reset.jlink

recover.jlink:
	echo "si 0\nt0\nsleep 1\ntck1\nsleep 1\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\ntck0\nsleep 100\nsi 1\nr\nexit\n" > $(OUTPUT_DIR)/recover.jlink

pin-reset.jlink:
	echo "device nrf51822\nw4 4001e504 2\nw4 40000544 1\nr\nexit\n" > $(OUTPUT_DIR)/pin-reset.jlink

erase-all: erase-all.jlink
	$(JLINK) $(OUTPUT_DIR)/erase-all.jlink

erase-all.jlink:
	echo "device nrf51822\nw4 4001e504 2\nw4 4001e50c 1\nw4 4001e514 1\nr\nexit\n" > $(OUTPUT_DIR)/erase-all.jlink

startdebug: stopdebug debug.jlink .gdbinit
	$(JLINKGDBSERVER) -single -if swd -speed 1000 -port $(GDB_PORT_NUMBER) &
	sleep 1
	$(GDB) $(ELF)

stopdebug:
	-killall $(JLINKGDBSERVER)

.gdbinit:
	echo "target remote localhost:$(GDB_PORT_NUMBER)\nmonitor flash download = 1\nmonitor flash device = nrf51822\nbreak main\nmon reset\n" > .gdbinit

debug.jlink:
	echo "Device nrf51822" > $(OUTPUT_DIR)/debug.jlink
		  
.PHONY: upload upload-softdevice erase-all startdebug stopdebug
