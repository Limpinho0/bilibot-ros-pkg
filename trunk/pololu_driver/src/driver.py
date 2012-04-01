#!/usr/bin/env python
import roslib; roslib.load_manifest('pololu_driver')
import rospy
import serial, time


from geometry_msgs.msg import Twist

def get_command(target):
    cmdbyte=chr(0x85)
    if target < 0:
      cmdbyte=chr(0x86)
    target=target*-1
    serialBytes = cmdbyte+chr(target & 0x1F)+chr((target >> 5) & 0x7F)
    return serialBytes
    
    

class PololuDriver(object):
    def __init__(self):
        rospy.init_node('pololu_driver')
        rospy.on_shutdown(self.ShutdownCallback)
        rospy.Subscriber("/cmd_vel",Twist, self.SteeringCallback)
        port=['/dev/ttyACM0','/dev/ttyACM3']
        self.ser=[]
        self.timeoutmax=1000
        self.timeout=self.timeoutmax
        try:
            self.ser.append(serial.Serial(port[0]))
            self.ser[0].open()
            self.ser[0].write(chr(0xAA))
            self.ser[0].flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr(rospy.get_name()+": Error opening or initialising port "+port[0])
            exit(1)
            
        try:
            self.ser.append(serial.Serial(port[1]))
            self.ser[1].open()
            self.ser[1].write(chr(0xAA))
            self.ser[1].flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr(rospy.get_name()+": Error opening or initialising port "+port[1])
            exit(1)
            
        self.IdentifyPorts()
	  
    def IdentifyPorts(self):
      #left wheel identifies as 0x98, right as 0x9c
      #this is based on driver model number, so should be changed...
      self.left=-1
      self.right=-1
      for i in [0,1]:
        self.ser[i].flush()
        self.ser[i].write(chr(0xc2))
        inbyte=self.ser[i].read()
        print inbyte
        if inbyte == '\x9c':
          self.right=i
        if inbyte == '\x98':
          self.left=i
        self.ser[i].read(); self.ser[i].read(); self.ser[i].read()
      if self.left == self.right or self.left<0 or self.right<0:
        rospy.logerr(rospy.get_name()+": error determining motor drivers ")
        exit(1)
      
      
      
    def cmdToLR(self,linear,angular):
        #normal vel: .5 linear, 1.0 angular
        #convert angular and linear to left and right
        #going left is positive angle
        lcmd=linear
        rcmd=linear
        lcmd-=angular/2.0 #if going left, left wheel is neg
        rcmd+=angular/2.0
        lcmd*=1500.0
        rcmd*=1500.0
        lcmd=max(-3200,min(1200,lcmd))
        rcmd=max(-3200,min(1200,rcmd))
        return int(lcmd),int(rcmd)

    def run(self):
      
        while not rospy.is_shutdown():
          time.sleep(0.02)
          self.timeout-=20
          if self.timeout<0:
            for i in [0,1]:
              self.ser[i].write(get_command(0))
            self.timeout=self.timeoutmax
            
          

    def ShutdownCallback(self):
        rospy.loginfo("Shutting down")
        if hasattr(self, 'ser'):
          for i in [0,1]:
            self.ser[i].write(get_command(0))
            self.ser[i].write(get_command(0))
            self.ser[i].write(chr(0xA2))
            self.ser[i].close()

    def SteeringCallback(self,data):
        self.timeout=self.timeoutmax
        lcmd,rcmd = self.cmdToLR(data.linear.x,data.angular.z)
        logstr=": motion cmd:  Linear="+str(data.linear.x)+" Angle="+str(data.angular.z)
        logstr+=" cmd=("+str(lcmd)+","+str(rcmd)+")"
        rospy.loginfo(rospy.get_name()+logstr )
        self.ser[self.left].write(get_command(lcmd))
        self.ser[self.right].write(get_command(rcmd))


#Init and run
PololuDriver().run()
