import dbus
import time
import pynotify
import socket, fcntl, struct
from gi.repository import Gio
from dbus.mainloop.glib import DBusGMainLoop
from ZeroconfService import ZeroconfService
import gobject
gobject.threads_init()
from dbus import glib

class BilibotNetworkService(object):

    BASE_KEY = "apps.bilibot"
    NM_CONNECTION_ACTIVATED = 2
    WLAN = "eth0"

    def __init__(self):
        glib.init_threads()

        self.system_bus = dbus.SystemBus()

        self.system_bus.add_signal_receiver(self.connection_handler,
                dbus_interface="org.freedesktop.NetworkManager.Connection.Active",
                signal_name=None)

        self.settings = Gio.Settings.new(self.BASE_KEY)

        DBusGMainLoop(set_as_default=True)

        loop = gobject.MainLoop()
        loop.run()

    def connection_handler(self, data=None):
        def ifconfig(ifname):
            ifreq = {'ifname': ifname}
            sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                ifreq['addr']    = _ifinfo(sock, 0x8915, ifname) # SIOCGIFADDR 
                ifreq['brdaddr'] = _ifinfo(sock, 0x8919, ifname) # SIOCGIFBRDADDR 
                ifreq['netmask'] = _ifinfo(sock, 0x891b, ifname) # SIOCGIFNETMASK 
                ifreq['hwaddr']  = _ifinfo(sock, 0x8927, ifname) # SIOCSIFHWADDR 
            except:
                pass
            sock.close()
            return ifreq

        def _ifinfo(sock, addr, ifname):
            iface = struct.pack('256s', ifname[:15])
            info  = fcntl.ioctl(sock.fileno(), addr, iface)
            if addr == 0x8927:
                hwaddr = []
                for char in info[18:24]:
                    hwaddr.append(hex(ord(char))[2:])
                return ':'.join(hwaddr)
            else:
                return socket.inet_ntoa(info[20:24])

        if (data is not None and 
            data[dbus.String("State")] == self.NM_CONNECTION_ACTIVATED):

            if (self.settings.get_boolean("configured-wifi")):
                return

            iface = ifconfig(self.WLAN)

            if (not 'addr' in iface):
                print "no addr in iface %s"%self.WLAN
                return

            print iface['addr']
            self.service = ZeroconfService(name="Bilibot",
                              port=11311, text="ROSIP=%s"%iface['addr'], stype="_ros._tcp")
            self.service.publish()

            self.settings.set_boolean("configured-wifi", True)

            if not pynotify.init("Basics"):
                print "missing notify bindings, can't notify user"
                return

            n = pynotify.Notification("Bilibot is online", "Bilibot is now on Wifi. You can now safely disconnect from VNC and remotely run apps through the thumbdrive.")
            n.show()

    def get_wireless_conn_info(self):
        for c in self.get_connections():
            proxy = self.system_bus.get_object('org.freedesktop.NetworkManagerUserSettings', c)
            iface = dbus.Interface(proxy, dbus_interface='org.freedesktop.NetworkManagerSettings.Connection')
            settings = iface.GetSettings()
            conn = settings['connection'] 
            if (conn['type'] == '802-11-wireless'):
                print "found wireless connection uuid: %s, name: %s"%(conn['uuid'], conn['id'])
                self.wireless_uuid = conn['uuid']
                self.wireless_id = conn['id']
                return
            print "no wireless iface connection found in NM"

    def get_connections(self):
        proxy = self.system_bus.get_object('org.freedesktop.NetworkManagerUserSettings', '/org/freedesktop/NetworkManagerSettings')
        iface = dbus.Interface(proxy, dbus_interface='org.freedesktop.NetworkManagerSettings')
        return iface.ListConnections() 


if __name__ == "__main__":
    bbns = BilibotNetworkService()

