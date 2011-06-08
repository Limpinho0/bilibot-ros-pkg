import dbus, gobject, avahi
import socket, fcntl, struct
import pynotify
from dbus import DBusException
from dbus.mainloop.glib import DBusGMainLoop

# Looks for ROS shares

TYPE = "_ros._tcp"

def service_resolved(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11):
    print '===================='
    print 'service resolved'
    print 'opt:', a1
    print 'opt:', a2
    print 'name:', a3
    print 'opt:', a4
    print 'opt:', a5
    print 'opt:', a6
    print 'opt:', a7
    print 'address:', a8
    print 'port:', a9
    print 'opt:', a10
    print 'opt:', a11
    #n = pynotify.Notification("Found Bilibot", "Configuring Bilibot for remote application support...")
    #n.show()
    file = open('/home/labuser/ros/setup.sh', 'a')
    #file.write("export ROS_MASTER_URI=http://%s:11311"%args)
    #n = pynotify.Notification("Bilibot Configured", "You can now run applications on the Bilibot")
    #n.show()

def print_error(*args):
    print 'error_handler'
    print args[0]

def myhandler(interface, protocol, name, stype, domain, flags):
    print "Found service '%s' type '%s' domain '%s' " % (name, stype, domain)

    #FIXME uncomment in prod version
    #if flags & avahi.LOOKUP_RESULT_LOCAL:
            # local service, skip
    #        pass

    server.ResolveService(interface, protocol, name, stype, 
        domain, avahi.PROTO_UNSPEC, dbus.UInt32(0), 
        reply_handler=service_resolved, error_handler=print_error)

loop = DBusGMainLoop()

bus = dbus.SystemBus(mainloop=loop)

server = dbus.Interface( bus.get_object(avahi.DBUS_NAME, '/'),
        'org.freedesktop.Avahi.Server')

sbrowser = dbus.Interface(bus.get_object(avahi.DBUS_NAME,
        server.ServiceBrowserNew(avahi.IF_UNSPEC,
            avahi.PROTO_UNSPEC, TYPE, 'local', dbus.UInt32(0))),
        avahi.DBUS_INTERFACE_SERVICE_BROWSER)

sbrowser.connect_to_signal("ItemNew", myhandler)

gobject.MainLoop().run()
