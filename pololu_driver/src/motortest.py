#!/usr/bin/python
import math, time, serial

def get_command(target):
	cmdbyte=chr(0x85)
        if target < 0:
	  cmdbyte=chr(0x86)
	  target=target*-1
        serialBytes = cmdbyte+chr(target & 0x7F)+chr((target >> 7) & 0x7F)
        return serialBytes

ser = serial.Serial('/dev/ttyACM0')
if not ser.isOpen():
  print "error opening serial port "
  exit(-1)

ser.write(chr(0xAA))
ser.flush()

i=0.0
while(i<2*math.pi):
        i = i+0.01
        #print math.sin(i)
        ser.write(get_command(int((math.sin(i)*3199))))

        time.sleep(0.002)

ser.write(get_command(0))
ser.write(chr(0xC2))
print ser.read()
print ser.read()
print ser.read()
print ser.read()

ser.close()
