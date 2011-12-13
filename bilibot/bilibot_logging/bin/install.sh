#!/bin/bash

# Run this from the directory containing it!
#
# USAGE: ./install.sh

origdir=$(pwd)
cd
pkgpath=$(pwd)/bin/logging
mkdir -p $pkgpath
cd $origdir
echo $(pwd)

echo "Installing logging scripts"


#the uploading scripts:
sed "s%##PATHHERE##%$pkgpath%g" < uploadall > $pkgpath/uploadall
sed "s%##PATHHERE##%$pkgpath%g" < uploadlog > $pkgpath/uploadlog
sed "s%##PATHHERE##%$pkgpath%g" < uploadlogs > $pkgpath/uploadlogs
sed "s%##PATHHERE##%$pkgpath%g" < postencode.py > $pkgpath/postencode.py
chmod +x $pkgpath/*


#Identifier - this tells us what robot is reporting
ident=$(grep -A20 "DEVNAME=/dev/sda\$" /var/log/udev | grep ID_SERIAL_SHORT | awk -F\= '{print $2}')
echo $ident >  $pkgpath/identifier


#now install the crontab:
