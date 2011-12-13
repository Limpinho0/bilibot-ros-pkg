rospack find bilibot_logging
svn up
echo $(grep -A20 "DEVNAME=/dev/sda\$" /var/log/udev | grep ID_SERIAL_SHORT | awk -F\= '{print $2}') > identifier
./uploadall