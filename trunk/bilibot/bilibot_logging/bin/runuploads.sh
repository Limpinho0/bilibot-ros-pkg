cd $(rospack find bilibot_logging)
svn up
pkgpath=##PATHHERE##

ident=$(grep -A20 "DEVNAME=/dev/sda\$" /var/log/udev | grep ID_SERIAL_SHORT | awk -F\= '{print $2}')
test -e $pkgpath/identifier || echo $ident >  $pkgpath/identifier

$pkgpath/uploadall