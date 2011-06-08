#!/usr/bin/python

import wx
import dbus, gobject, avahi
import pynotify
import os 
from dbus.mainloop.glib import DBusGMainLoop

class MasterSelectorFrame(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, (550, 350))

        panel = wx.Panel(self, -1)

        self.master_list = wx.ListBox(panel, 26, (50,50), (450, 230), [], wx.LB_SINGLE)

        #self.master_list.SetSelection(0)

        btn = wx.Button(panel, wx.ID_CLOSE, 'Close', (225, 300))

        self.Bind(wx.EVT_BUTTON, self.OnClose, id=wx.ID_CLOSE)
        self.Bind(wx.EVT_LISTBOX, self.OnSelect, id=26)

    def OnClose(self, event):
        self.Close()

    def OnSelect(self, event):
        index = event.GetSelection()
        master = self.master_list.GetString(index)
        settings = master.split(',')
        name = settings[0].strip()
        addr = settings[1].strip()
        port = settings[2].strip()

        file = open('%s/ros/master.sh'%os.getenv("HOME"), 'w')
        file.write("export ROS_MASTER_URI=http://%s:%s"%(addr,port))
        file.close()

        n = pynotify.Notification("New ROS_MASTER=%s"%addr, "Please reopen all terminals for new settings to take effect...")
        n.show()

class MasterSelectorApp(wx.App):
    def OnInit(self):
        self.frame = MasterSelectorFrame(None, -1, 'ROS Master Selector')
        self.frame.Centre()
        self.frame.Show(True)

        loop = DBusGMainLoop()
        bus = dbus.SystemBus(mainloop=loop)

        self.server = dbus.Interface( bus.get_object(avahi.DBUS_NAME, '/'),
                                    'org.freedesktop.Avahi.Server')

        sbrowser = dbus.Interface(bus.get_object(avahi.DBUS_NAME,
                                    self.server.ServiceBrowserNew(avahi.IF_UNSPEC,
                                    avahi.PROTO_UNSPEC, "_ros._tcp", 'local', dbus.UInt32(0))),
                                    avahi.DBUS_INTERFACE_SERVICE_BROWSER)

        sbrowser.connect_to_signal("ItemNew", self.ServiceFound)

        return True


    def ServiceFound(self, interface, protocol, name, stype, domain, flags):
        def print_error(*args):
            print 'error_handler'
            print args[0]

        #if flags & avahi.LOOKUP_RESULT_LOCAL:
            # local service, skip
            #pass

        self.server.ResolveService(interface, protocol, name, stype, 
                                    domain, avahi.PROTO_UNSPEC, dbus.UInt32(0), 
                                    reply_handler=self.ServiceResolved, error_handler=print_error)

    def ServiceResolved(self, *args):
        name = args[2]
        addr = args[7]
        port = args[8]
        self.frame.master_list.Insert("%s, %s, %s"%(name,addr,port), 0)

app = MasterSelectorApp(0)
app.MainLoop()
