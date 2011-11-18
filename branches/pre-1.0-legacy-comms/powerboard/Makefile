CC=/usr/bin/avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=atmega128
OBJ2HEX=/usr/bin/avr-objcopy 
TARGET=adcread

all : $(TARGET).hex blink2.hex
#music.hex music2.hex blink.hex music3.hex servo.hex servo2.hex music4.c servo3.hex box.hex stepper.hex testy.hex car.hex sonic.hex display.hex testcar.hex cardreader.hex credit.hex tbutton.hex doorlock.hex reader.hex

#$(TARGET).obj : $(TARGET).o
#    $(CC) $(CFLAGS) $< -o $@

#%$(TARGET).hex : %$(TARGET).obj
#    $(OBJ2HEX) -R .eeprom -O ihex $< $@

decode: decoding2.cpp
	g++ -Wall -g -o decode decoding2.cpp


%.hex : %.out
	$(OBJ2HEX) -R .eeprom -O ihex $< $@
%.out : %.o
	$(CC) $(CFLAGS) -o $@ -Wl,-Map,$*.map $<
%.o : %.c
	$(CC) $(CFLAGS) -Os -c $<


clean :
	rm -f *.hex *.obj *.o


#$(TARGET).hex : $(TARGET).out
#	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET).out $(TARGET).hex
#$(TARGET).out : $(TARGET).o
#	$(CC) $(CFLAGS) -o $(TARGET).out -Wl,-Map,$(TARGET).map $(TARGET).o
#$(TARGET).o : $(TARGET).c
#	$(CC) $(CFLAGS) -Os -c $(TARGET).c


