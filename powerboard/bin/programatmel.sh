#program the atmel by resetting it, then running avrdude
#you will need to copy 56-ftdi-usb.rules to /etc/udev/rules.d

#TODO: test lenght to see if rospack failed
dir=$(rospack find bilibot_node)/../../powerboard/bin

test $1 || echo "Error: no hexfile given"
test $1 || exit

# 
# ls $dir
# $dir/../testprog
# 
# 
# exit

#reset the atmel:
echo "$dir/togglects"
$dir/togglects &
sleep 3.0

#program the chip:
echo "avrdude -p usb1287 -P /dev/ttyACM0 -c avr109 -U flash:w:$1"
/usr/bin/avrdude -p usb1287 -P /dev/ttyACM0 -c avr109 -U flash:w:$1

sleep 2

#now reset the ftdi chip, so it re-enumerates:
echo "$dir/usbreset2"
$dir/usbreset2