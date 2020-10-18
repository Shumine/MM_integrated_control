#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,sys, signal
from std_msgs.msg import Int64
from geometry_msgs.msg import WrenchStamped, Wrench

import os

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

def ft_sensor_val(msg):
    global fx
    global fy
    global fz

    fx = msg.wrench.force.x
    fy = msg.wrench.force.y
    fz = msg.wrench.force.z




if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library



if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    sub = rospy.Subscriber('/optoforce_node/OptoForceWrench', WrenchStamped, ft_sensor_val)



    # Control table address
    ADDR_PRO_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION      = 564
    ADDR_PRO_GOAL_CURRENT       = 550 
    ADDR_PRO_PRESENT_CURRENT    = 574
    OPERATING_MODE              = 11

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL_ID                      = 1                 # Dynamixel ID : 1
    BAUDRATE                    = 9600             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
   




    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    # Set current control mode
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, OPERATING_MODE, 0)

    pub = rospy.Publisher('gripper_val', Int64, queue_size=10)
    rospy.init_node('optoforce_gripper', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    while 1:
        packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, -500)

        print "Opening Gripper!"





    # Close port
    portHandler.closePort()
