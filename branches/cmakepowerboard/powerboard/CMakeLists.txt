cmake_minimum_required (VERSION 2.8)
project (POWERBOARD) 

SET(CMAKE_SYSTEM_NAME Generic)

# specify the cross compiler
SET(CMAKE_C_COMPILER   /usr/bin/avr-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/avr-g++)

SET(CSTANDARD "-std=gnu99")
#SET(CDEBUG "-gstabs")
SET(CWARN "-Wall -Wstrict-prototypes")
SET(CTUNING "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT "-Os")
SET(CINCS "-I. -I./sdk")
SET(CMCU "-mmcu=at90usb1287")
SET(CDEFS "-DF_CPU=16000000")


SET(CFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT}")

SET(CMAKE_C_FLAGS ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})

add_definitions(-gdwarf-2 -DF_CPU=16000000UL -DF_USB=16000000UL -DBOARD=BOARD_BILIBOT -DARCH=ARCH_AVR8 -D USB_DEVICE_ONLY -D FIXED_CONTROL_ENDPOINT_SIZE=8 -D FIXED_NUM_CONFIGURATIONS=1 -D USE_FLASH_DESCRIPTORS -D USE_STATIC_OPTIONS=\"(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)\")

#add_definitions(-mmcu=at90usb1287 -I. -gdwarf-2 -DF_CPU=16000000UL -DF_USB=16000000UL -DBOARD=BOARD_BILIBOT -DARCH=ARCH_AVR8 -D USB_DEVICE_ONLY -D FIXED_CONTROL_ENDPOINT_SIZE=8 -D FIXED_NUM_CONFIGURATIONS=1 -D USE_FLASH_DESCRIPTORS -D USE_STATIC_OPTIONS=\"(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)\")
#add_definitions(-Os -funsigned-char -funsigned-bitfields -ffunction-sections -fno-inline-small-functions -fpack-struct -fshort-enums -fno-strict-aliasing -Wall -Wstrict-prototypes -Wa, -I./ -Isdk/ -std=c99 -MMD -MP )

add_library(Descriptors Descriptors.c)
add_library(wireformat sdk/wireformat.c)
add_library(lufacore LUFA/Drivers/USB/Core/AVR8/Device_AVR8.c 
			LUFA/Drivers/USB/Core/AVR8/Endpoint_AVR8.c
			LUFA/Drivers/USB/Core/AVR8/Host_AVR8.c
			LUFA/Drivers/USB/Core/AVR8/Pipe_AVR8.c
			LUFA/Drivers/USB/Core/AVR8/USBController_AVR8.c
			LUFA/Drivers/USB/Core/AVR8/USBInterrupt_AVR8.c
			LUFA/Drivers/USB/Class/Device/CDC.c 
			LUFA/Drivers/USB/Core/ConfigDescriptor.c
			LUFA/Drivers/USB/Core/DeviceStandardReq.c
			LUFA/Drivers/USB/Core/Events.c
			LUFA/Drivers/USB/Core/EndpointStream.c
			LUFA/Drivers/USB/Core/HostStandardReq.c
			LUFA/Drivers/USB/Core/PipeStream.c
			LUFA/Drivers/USB/Core/USBTask.c
			LUFA/Drivers/USB/Class/Common/HIDParser.c)

# powerboard.o Descriptors.o sdk/wireformat.o 
# LUFA/Drivers/USB/Core/AVR8/Device_AVR8.o 
# LUFA/Drivers/USB/Core/AVR8/Endpoint_AVR8.o 
# LUFA/Drivers/USB/Core/AVR8/Host_AVR8.o 
# LUFA/Drivers/USB/Core/AVR8/Pipe_AVR8.o 
# LUFA/Drivers/USB/Core/AVR8/USBController_AVR8.o 
# LUFA/Drivers/USB/Core/AVR8/USBInterrupt_AVR8.o 

#add_library(lufacore LUFA/Drivers/USB/Core/ConfigDescriptor.c
#			LUFA/Drivers/USB/Core/DeviceStandardReq.c
#			LUFA/Drivers/USB/Core/Events.c
#			LUFA/Drivers/USB/Core/EndpointStream.c
#			LUFA/Drivers/USB/Core/HostStandardReq.c
#			LUFA/Drivers/USB/Core/PipeStream.c
#			LUFA/Drivers/USB/Core/USBTask.c)

add_definitions(-gdwarf-2 -DF_CPU=16000000UL -DF_USB=16000000UL -DBOARD=BOARD_BILIBOT -DARCH=ARCH_AVR8 -D USB_DEVICE_ONLY -D FIXED_CONTROL_ENDPOINT_SIZE=8 -D FIXED_NUM_CONFIGURATIONS=1 -D USE_FLASH_DESCRIPTORS -D USE_STATIC_OPTIONS=\"(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)\")
			
add_executable( powerboard.elf powerboard.c)
target_link_libraries(powerboard.elf lufacore wireformat Descriptors)

# LUFA/Drivers/USB/Core/ConfigDescriptor.o 
# LUFA/Drivers/USB/Core/DeviceStandardReq.o 
# LUFA/Drivers/USB/Core/Events.o 
# LUFA/Drivers/USB/Core/EndpointStream.o 
# LUFA/Drivers/USB/Core/HostStandardReq.o 
# LUFA/Drivers/USB/Core/PipeStream.o 
# LUFA/Drivers/USB/Core/USBTask.o 



#unneeded
# LUFA/Drivers/USB/Class/Common/HIDParser.o 
# LUFA/Drivers/USB/Class/Device/Audio.o 
# LUFA/Drivers/USB/Class/Device/CDC.o 
# LUFA/Drivers/USB/Class/Device/HID.o 
# LUFA/Drivers/USB/Class/Device/MassStorage.o 
# LUFA/Drivers/USB/Class/Device/MIDI.o 
# LUFA/Drivers/USB/Class/Device/RNDIS.o 
# LUFA/Drivers/USB/Class/Host/CDC.o 
# LUFA/Drivers/USB/Class/Host/HID.o 
# LUFA/Drivers/USB/Class/Host/MassStorage.o 
# LUFA/Drivers/USB/Class/Host/MIDI.o 
# LUFA/Drivers/USB/Class/Host/Printer.o 
# LUFA/Drivers/USB/Class/Host/RNDIS.o 
# LUFA/Drivers/USB/Class/Host/StillImage.o


add_custom_command(TARGET powerboard.elf POST_BUILD
				   COMMAND avr-objcopy -O ihex -R .eeprom powerboard.elf powerboard.hex)