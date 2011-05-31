import dbus, gobject, avahi
import socket, fcntl, struct
import pynotify
from dbus import DBusException
from dbus.mainloop.glib import DBusGMainLoop

# Looks for ROS shares

TYPE = "_ros._tcp"

def service_resolved(*args):
    print 'service resolved'
    print 'port:', args
    n = pynotify.Notification("Found Bilibot", "Configuring Bilibot for remote application support...")
    n.show()
    file = open('/home/bilibot/ros/setup.sh', 'a')
    file.write("export ROS_MASTER_URI=http://%s:11311"%args)
    n = pynotify.Notification("Bilibot Configured", "You can now run applications on the Bilibot")
    n.show()

def print_error(*args):
    print 'error_handler'
    print args[0]

def myhandler(interface, protocol, name, stype, domain, flags):
    print "Found service '%s' type '%s' domain '%s' " % (name, stype, domain)

    if flags & avahi.LOOKUP_RESULT_LOCAL:
            # local service, skip
            pass

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
