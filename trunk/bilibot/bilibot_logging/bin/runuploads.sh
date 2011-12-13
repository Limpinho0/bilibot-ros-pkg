cd $(rospack find bilibot_logging)
svn up

ident=$(grep -A20 "DEVNAME=/dev/sda\$" /var/log/udev | grep ID_SERIAL_SHORT | awk -F\= '{print $2}')
test -e $(rospack find bilibot_logging)/identifier || echo $ident >  $(rospack find bilibot_logging)/identifier

$(rospack find bilibot_logging)/bin/uploadall