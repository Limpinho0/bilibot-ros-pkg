/*
 *  finddev.c - Find a device node given (type,major,minor) using libudev
 *  Copyright (C) 2009  Andy Walls <awa...@radix.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  Compilation:
 *      gcc -o finddev finddev.c -Wall -ludev
 *
 *  Example invocation (/dev/null on my system):
 *      ./finddev -c -M 1 -m 3
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <cstring>
#include <string>
#include <sys/stat.h>
extern "C" {
#include <libudev.h>
}
//#include "ros/ros.h"
//#include "device_finder/FindPort.h"
// #include <hokuyo/hokuyo.h>
using namespace std;

// string getID(string device){
// //	cout<<"looking for device..."<<device.c_str()<<endl;
// 	hokuyo::Laser laser;
// 	  for (int retries = 10; retries; retries--)
// 	  {
// 		try
// 		{
// 		  laser.open(device.c_str());
// 		  std::string device_id = laser.getID();
// //			printf("Device at %s has ID ", device.c_str());
// //		  printf("%s\n", device_id.c_str());
// 		  laser.close();
// 		  return device_id;
// 		}
// 		catch (hokuyo::Exception &e)
// 		{
// 		  laser.close();
// 		}
// 		sleep(1);
// 	  }
// 	  return "";
// }

class devinfo{
	string dname;
	string name; // the  '/dev/tty...' path you expect
	string path; //crazy path defined by system
	string subsystem;  //like usb, tty, block, etc
	string vendor;
	string qproperty;
	string qpropertyname;
	string serial;

	const char * getproperty(struct udev_device *udev_dev,string prop){
        struct udev_list_entry *udev_list_entry;
		udev_list_entry = udev_list_entry_get_by_name (udev_device_get_properties_list_entry (udev_dev),prop.c_str());
		if(udev_list_entry!= NULL)
			return udev_list_entry_get_value (udev_list_entry);
		return "";
	}

public:

	devinfo(struct udev_device *udev_dev,string qp="none"){

		name=udev_device_get_sysname(udev_dev);
		path=udev_device_get_syspath(udev_dev);
		subsystem=udev_device_get_subsystem (udev_dev);

        //get vendor_id property.  Not all devices will have this
//        udev_list_entry = udev_list_entry_get_by_name (udev_device_get_properties_list_entry (udev_dev),"ID_VENDOR");
//        if(udev_list_entry!= NULL)
//        	vendor=udev_list_entry_get_value (udev_list_entry);
        dname=getproperty(udev_dev,"DEVNAME");
        vendor=getproperty(udev_dev,"ID_VENDOR");
//         if(vendor.compare("Hokuyo_Data_Flex_for_USB")==0 && subsystem.compare("tty")==0)
//         	serial=getID(dname);
        if(vendor.compare("FTDI")==0)
        	serial=getproperty(udev_dev,"ID_SERIAL_SHORT");


        if(qp.compare("none")){
        	qproperty=getproperty(udev_dev,qp);
        	qpropertyname=qp;
//        	udev_list_entry = udev_list_entry_get_by_name (udev_device_get_properties_list_entry (udev_dev),qp.c_str());
//            if(udev_list_entry!= NULL)
//            	qproperty=udev_list_entry_get_value (udev_list_entry);
        }

	}
	devinfo(){}
	bool matchesvendor(string v){
		return vendor.compare(v)==0;
	}
	bool matchessubsytem(string v){
		return subsystem.compare(v)==0;
	}
	bool hasVendor(){
		return vendor.size()>0;
	}
	bool matchesvendor(const char *v){
		return vendor.compare(v)==0;
	}
	bool hasSerial(string s){
		return serial.size() > 0 && serial.compare(s)==0;
	}

	void print(){
		string fpath=name.insert(0,"/dev/");
		cout<<"device: "<<setw(15)<<left<<name<<"  subsystem: "<<setw(15)<<left<<subsystem<<"  vendor: "<<setw(15)<<left<<vendor;
		cout<<"path: "<<setw(15)<<left<<dname;
//		if(matchesvendor("Hokuyo_Data_Flex_for_USB") && subsystem.compare("tty")==0)
		if(serial.size()>0)
			cout<<"  serial number: "<<serial;
		if(qproperty.size()>0)
			cout<<qpropertyname<<": "<<qproperty;
		cout<<endl;
	}
	string getName(){
		return name;
	}

	void printsmall(){
		string fpath=name.insert(0,"/dev/");
		cout<<"device: "<<setw(15)<<left<<name;
//		if(matchesvendor("Hokuyo_Data_Flex_for_USB") && subsystem.compare("tty")==0)
		if(serial.size()>0)
			cout<<"  serial number: "<<serial;
		if(qproperty.size()>0)
			cout<<qpropertyname<<": "<<qproperty;
		cout<<endl;
	}
};


void listall(bool listnonvendor,string qproperty)
{
	cout<<"Listing all devices:"<<endl;
    struct udev *udev;
    struct udev_device *udev_dev;
    struct udev_enumerate *udev_enum;
    const char *s;
    struct udev_list_entry *udev_list_entry, *udev_list_entry2;
    vector<devinfo> devices;
    devinfo devi;
    udev = udev_new();
    if (udev == NULL) {
            fprintf(stderr, "udev_new() failed\n");
            return ;
    }
    string path,prefix=udev_get_dev_path(udev);

    prefix.append("/");
//    printf("Device directory path: '%s'\n", udev_get_dev_path(udev));

    udev_enum = udev_enumerate_new(udev);
//    udev_enumerate_add_match_subsystem (udev_enum,"tty");
    udev_enumerate_scan_devices(udev_enum);
    //now get a listing of all devices reporting to the system:
    udev_list_entry_foreach(udev_list_entry, udev_enumerate_get_list_entry(udev_enum)) {
            udev_dev = udev_device_new_from_syspath(udev,udev_list_entry_get_name(udev_list_entry));
            if (udev_dev == NULL) {
                    fprintf(stderr, "udev_device_new_from_syspath() failed\n");
                    udev_unref(udev);
                    return ;
            }
            devices.push_back(devinfo(udev_dev,qproperty));
            udev_device_unref(udev_dev);
    }
    for(int i=0;i<devices.size();i++){
        path=devices[i].getName().insert(0,prefix);
		if(listnonvendor || devices[i].hasVendor())
			devices[i].print();
    }

    udev_unref(udev);

  return ;
}


void listftdi(bool listnonvendor,string qproperty)
{
	cout<<"Listing all devices:"<<endl;
    struct udev *udev;
    struct udev_device *udev_dev;
    struct udev_enumerate *udev_enum;
    const char *s;
    struct udev_list_entry *udev_list_entry, *udev_list_entry2;
    vector<devinfo> devices;
    devinfo devi;
    udev = udev_new();
    if (udev == NULL) {
            fprintf(stderr, "udev_new() failed\n");
            return ;
    }
    string path,prefix=udev_get_dev_path(udev);

    prefix.append("/");
//    printf("Device directory path: '%s'\n", udev_get_dev_path(udev));

    udev_enum = udev_enumerate_new(udev);
//    udev_enumerate_add_match_subsystem (udev_enum,"tty");
    udev_enumerate_scan_devices(udev_enum);
    //now get a listing of all devices reporting to the system:
    udev_list_entry_foreach(udev_list_entry, udev_enumerate_get_list_entry(udev_enum)) {
            udev_dev = udev_device_new_from_syspath(udev,udev_list_entry_get_name(udev_list_entry));
            if (udev_dev == NULL) {
                    fprintf(stderr, "udev_device_new_from_syspath() failed\n");
                    udev_unref(udev);
                    return ;
            }
            devices.push_back(devinfo(udev_dev,qproperty));
            udev_device_unref(udev_dev);
    }
    for(int i=0;i<devices.size();i++){
        path=devices[i].getName().insert(0,prefix);
		if(listnonvendor || devices[i].hasVendor())
		   if(devices[i].matchesvendor("FTDI") && devices[i].matchessubsytem("tty"))
			devices[i].printsmall();
    }

    udev_unref(udev);

  return ;
}

void getmydev(string qproperty)
{
    struct udev *udev;
    struct udev_device *udev_dev;
    struct udev_enumerate *udev_enum;
    const char *s;
    struct udev_list_entry *udev_list_entry, *udev_list_entry2;
    vector<devinfo> devices;
    devinfo devi;
    udev = udev_new();
    if (udev == NULL) {
            fprintf(stderr, "udev_new() failed\n");
            return ;
    }
    string path,prefix=udev_get_dev_path(udev);

    prefix.append("/");
//    printf("Device directory path: '%s'\n", udev_get_dev_path(udev));

    udev_enum = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem (udev_enum,"tty");
    udev_enumerate_scan_devices(udev_enum);
    //now get a listing of all devices reporting to the system:
    udev_list_entry_foreach(udev_list_entry, udev_enumerate_get_list_entry(udev_enum)) {
            udev_dev = udev_device_new_from_syspath(udev,udev_list_entry_get_name(udev_list_entry));
            if (udev_dev == NULL) {
                    fprintf(stderr, "udev_device_new_from_syspath() failed\n");
                    udev_unref(udev);
                    return ;
            }
            devices.push_back(devinfo(udev_dev));
            udev_device_unref(udev_dev);
    }
    for(int i=0;i<devices.size();i++){
        path=devices[i].getName().insert(0,prefix);
		if(devices[i].hasSerial(qproperty)){
			cout<<"/dev/"<<devices[i].getName();
		    udev_unref(udev);
//			devices[i].print();
		    return;
		}
    }
    cout<<"No_Device_Found"<<endl;
    udev_unref(udev);

  return ;
}


int main(int argc, char **argv)
{
	bool getdev=false;
	bool listnonvendor=false;
	int argp=1;
	string qproperty="none";
	while (argp<argc){
		if(strcmp(argv[argp],"-v")==0){
			listnonvendor=true;
			argp++;
			continue;
		}
		if(strcmp(argv[argp],"-s")==0){
			qproperty=string(argv[argp+1]);
			getdev=true;
			argp+=2;
			continue;
		}
		if(strcmp(argv[argp],"-f")==0){
			listftdi(listnonvendor,"none");
			return 0;
		}
		if(argc-argp>1 && strcmp(argv[argp],"-p")==0){
			qproperty=string(argv[argp+1]);
			argp+=2;
			continue;
		}
		argp++;
	}
  if(getdev)
	  getmydev(qproperty);
  else
	  listall(listnonvendor,qproperty);

  return 0;
}

