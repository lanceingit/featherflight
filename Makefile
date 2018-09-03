#Program name
PROG = ff


CROSS_COMPILE     ?= arm-none-eabi-

############### Location configuration ################
LOAD_ADDRESS = 0x8004000
LINKER_DIR = make/linker
BUILD_DIR = make/build
BIN_DIR = bin
LIB = src/bsp
MAVLINKLIB = src/link/mavlink

################ Build configuration ##################
# St Lib
VPATH += $(LIB)
VPATH += $(LIB)/STM32F4xx_StdPeriph_Driver/src
VPATH += $(LIB)/CMSIS/CM4/DeviceSupport/ST/STM32F4xx/startup/gcc_ride7
VPATH += $(LIB)/STM32_USB_Device_Library/Core/src
VPATH += $(LIB)/STM32_USB_Device_Library/Class/cdc/src
VPATH += $(LIB)/STM32_USB_OTG_Driver/src
CRT0_CF2 = startup_stm32f427x.o system_stm32f4xx.o

ST_OBJ=
ST_OBJ+=misc.o
#ST_OBJ+=stm32f4xx_adc.o
#ST_OBJ+=stm32f4xx_bkp.o
#ST_OBJ+=stm32f4xx_can.o
#ST_OBJ+=stm32f4xx_crc.o
#ST_OBJ+=stm32f4xx_dac.o
#ST_OBJ+=stm32f4xx_dbgmcu.o
ST_OBJ+=stm32f4xx_dma.o
#ST_OBJ+=stm32f4xx_exti.o
#ST_OBJ+=stm32f4xx_flash.o
#ST_OBJ+=stm32f4xx_fsmc.o
ST_OBJ+=stm32f4xx_gpio.o
#ST_OBJ+=stm32f4xx_i2c.o
#ST_OBJ+=stm32f4xx_iwdg.o
#ST_OBJ+=stm32f4xx_pwr.o
ST_OBJ+=stm32f4xx_rcc.o
#ST_OBJ+=stm32f4xx_rtc.o
#ST_OBJ+=stm32f4xx_sdio.o
#ST_OBJ+=stm32f4xx_spi.o
ST_OBJ+=stm32f4xx_tim.o
ST_OBJ+=stm32f4xx_usart.o
#ST_OBJ+=stm32f4xx_misc.o
#ST_OBJ+=stm32f4xx_wwdg.o
ST_OBJ+=stm32f4xx_syscfg.o


# USB obj
ST_OBJ += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ += usbd_ioreq.o usbd_req.o usbd_core.o usbd_cdc_core.o


# sources
VPATH += src src/bsp src/bsp/usb 
VPATH += src/driver src/driver/sensor src/module src/link src/utils 
VPATH += src/kernel/FreeRTOS src/kernel/FreeRTOS/portable/MemMang src/kernel/FreeRTOS/portable/Common \
         src/kernel/FreeRTOS/portable/GCC/ARM_CM4F   
VPATH += src/utils/libcxx
VPATH += src/utils/mathlib/math src/utils/matrix src/utils/geo src/utils/geo_lookup


############### Source files configuration ################

# launch
PROJ_OBJ = main.o
PROJ_OBJ += copter.o  

# driver
PROJ_OBJ += timer.o  
PROJ_OBJ += serial.o
#PROJ_OBJ += hmc5883.o mpu6050.o mpu9250.o ms5611.o

# usb
PROJ_OBJ += usb_bsp.o usbd_desc.o usbd_cdc_vcp.o usbd_usr.o

#kernel
PROJ_OBJ += tasks.o list.o queue.o port.o heap_4.o

# module
PROJ_OBJ += mavlink_func.o 
PROJ_OBJ += att_est_q.o mix_estimator.o
PROJ_OBJ += cli.o 

#link
PROJ_OBJ += link_mavlink.o 


# utils
PROJ_OBJ += fifo.o mm.o eprintf.o
PROJ_OBJ += LowPassFilter2p.o
PROJ_OBJ += libstubs.o
PROJ_OBJ += geo.o geo_mag_declination.o 


OBJ = $(PROJ_OBJ) $(ST_OBJ) $(CRT0_CF2)


############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CPP = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)g++
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

INCLUDES  = -Isrc
INCLUDES += -Isrc/bsp -Isrc/bsp/usb -Isrc/driver -Isrc/driver/sensor \
		    -Isrc/module -Isrc/link -Isrc/math -Isrc/utils 
INCLUDES += -Isrc/utils/mathlib -Isrc/utils/mathlib/math -Isrc/utils/matrix \
 		    -Isrc/utils/geo -Isrc/utils/geo_lookup		    
INCLUDES += -I$(MAVLINKLIB) -I$(MAVLINKLIB)/common
INCLUDES += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(LIB)/CMSIS/CM4/DeviceSupport/ST/STM32F4xx
INCLUDES += -I$(LIB)/CMSIS/CM4/CoreSupport
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Class/cdc/inc
INCLUDES += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES += -Isrc/kernel -Isrc/kernel/FreeRTOS/include -Isrc/kernel/FreeRTOS/portable/GCC/ARM_CM4F


PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -fno-math-errno -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP -D__FPU_USED=1 -D__USE_C99_MATH

#Flags required by the ST library
STFLAGS = -DSTM32F427X -DUSE_STDPERIPH_DRIVER -DUSE_USB_OTG_FS


# Fail on warnings
CFLAGS += -O0 -g3 -Werror

CFLAGS += $(PROCESSOR) $(INCLUDES) $(STFLAGS)

CFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BUILD_DIR)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion

CCFLAGS += $(CFLAGS) -std=c99

CPPFLAGS += $(CFLAGS) -std=gnu++11

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS = --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(BIN_DIR)/$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority 

LDFLAGS += -T $(LINKER_DIR)/FLASH.ld


#Where to compile the .o
VPATH += $(BUILD_DIR)

#Dependency files to include
DEPS := $(foreach o,$(OBJ),$(BUILD_DIR)/dep/$(o).d)


#################### Targets ###############################


all: compile size
compile: $(PROG).hex $(PROG).bin $(PROG).dfu $(PROG).px4

size: compile
	@$(SIZE) -B $(BIN_DIR)/$(PROG).elf
	
$(PROG).hex: $(PROG).elf
	$(OBJCOPY) $(BIN_DIR)/$< -O ihex $(BIN_DIR)/$@

$(PROG).px4: $(PROG).bin
	python tools/px_mkfw.py --prototype tools/px4fmu-v4.prototype --image $(BIN_DIR)/$< > $(BIN_DIR)/$@
	
$(PROG).dfu: $(PROG).bin
	python tools/dfu-convert.py -b $(LOAD_ADDRESS):$(BIN_DIR)/$< $(BIN_DIR)/$@

$(PROG).bin: $(PROG).elf
	$(OBJCOPY) $(BIN_DIR)/$< -O binary --pad-to 0 $(BIN_DIR)/$@


$(PROG).elf: $(OBJ) $(CPP_OBJ)
	$(LD) $(LDFLAGS) $(foreach o,$(OBJ),$(BUILD_DIR)/$(o)) -lm -o $(BIN_DIR)/$@
	

.c.o:
	$(CC) $(CCFLAGS) -c $< -o $(BUILD_DIR)/$@

.cpp.o:
	$(CPP) $(CPPFLAGS) -c $< -o $(BUILD_DIR)/$@
	
.S.o:
	$(CC) $(CCFLAGS) -c $< -o $(BUILD_DIR)/$@


.s.o:
	$(AS) $(ASFLAGS) $< -o $(BUILD_DIR)/$@

upload:$(PROG).px4
	python tools/px_uploader.py --port /dev/ttyACM0 $(BIN_DIR)/$<

clean:
	rm -f $(BIN_DIR)/$(PROG).elf $(BIN_DIR)/$(PROG).hex $(BIN_DIR)/$(PROG).px4 \
	$(BIN_DIR)/$(PROG).bin $(BIN_DIR)/$(PROG).dfu $(BIN_DIR)/$(PROG).map \
	$(BUILD_DIR)/dep/*.d $(BUILD_DIR)/*.o


