#!/usr/bin/python

import time
import gobject
import avahi
import dbus
import socket

class ROSDiscoveryService(object):
    def __init__(self):
        self.publish()

        try:
            loop = gobject.MainLoop()
            loop.run()
        except (KeyboardInterrupt, SystemExit):
            print "\nexiting..."

    def __del__(self):
        self.unpublish()

    def publish(self):
        bus = dbus.SystemBus()
        server = dbus.Interface(
                         bus.get_object(
                                 avahi.DBUS_NAME,
                                 avahi.DBUS_PATH_SERVER),
                        avahi.DBUS_INTERFACE_SERVER)

        g = dbus.Interface(
                    bus.get_object(avahi.DBUS_NAME,
                                   server.EntryGroupNew()),
                    avahi.DBUS_INTERFACE_ENTRY_GROUP)

        g.AddService(avahi.IF_UNSPEC, avahi.PROTO_UNSPEC,dbus.UInt32(0),
                     socket.gethostname(), "_ros._tcp", "", "",
                     dbus.UInt16("11311"), "")

        g.Commit()
        self.group = g

    def unpublish(self):
        self.group.Reset()

if __name__ == "__main__":
    rds = ROSDiscoveryService()
