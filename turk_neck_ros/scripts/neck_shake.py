#!/usr/bin/env python
# license removed for brevity
from __future__ import print_function
import rospy
from std_msgs.msg import String
import os
import sys
import time

# init
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import termios, fcntl, sys, os
    from select import select
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)

    def getch():
        new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        return ch

    def kbhit():
        new_term[3] = (new_term[3] & ~(termios.ICANON | termios.ECHO))
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            dr,dw,de = select([sys.stdin], [], [], 0)
            if dr != []:
                return 1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            sys.stdout.flush()

        return 0

from dynamixel_sdk import port_handler
from dynamixel_sdk import packet_handler
from dynamixel_sdk import robotis_def as rd

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting
BAUDRATE                    = 57600
DEVICENAME                  = sys.argv[1]

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
MAX_POSITION_VALUE          = 0
DXL_MOVING_STATUS_THRESHOLD = 20
EXT_POSITION_CONTROL_MODE   = 4
ACCEL                       = 10000
VELOCITY                    = 30
SHAKE_DIST                  = 200

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20

# Control table address
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PROF_ACCEL             = 108
ADDR_PROF_VELOCITY          = 112




def init_servo(id):
    # Set operating mode to extended position control mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
    if dxl_comm_result != rd.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode for S" + str(id))

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != rd.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected to S" + str(id))

# Moves the neck to the initial position, returns the servo's internal position
def raise_neck(id):
    print("OwO! \n") #debug
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)

    goal_pos = dxl_present_position + 1024

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_ACCEL, ACCEL)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_VELOCITY, VELOCITY)


    while abs(dxl_present_position - goal_pos) > 100:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_pos)
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
        time.sleep(0.1)
        if kbhit():
            c = getch()
            if c == chr(ESC_ASCII_VALUE):
                print("\nNy loppuu!")
                break
    print("OwO DONE!") #debug
    return dxl_present_position

# Moves the neck around to scan for stuff
def scanmode(id, dxl_angle, zpos):
    print("Scanning!\n")

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_ACCEL, ACCEL)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PROF_VELOCITY, VELOCITY)
    goal_pos = zpos + dxl_angle
    state_flag = True
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_pos) #move to max position
    while True:
        dxl_after_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION) #read after position
        #print(dxl_after_pos)
        #print(goal_pos)
        if abs(dxl_after_pos - goal_pos) < 100:
            #print("Heythere!") #debug
            if state_flag:
                goal_pos = goal_pos - dxl_angle*2
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_pos) #move to max position
                state_flag = not state_flag
                #time.sleep(0.2)
            else:
                goal_pos = goal_pos + dxl_angle*2
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_pos) #move to other side position
                state_flag = not state_flag
                #time.sleep(0.2)
        if kbhit():
            c = getch()
            if c == chr(ESC_ASCII_VALUE):
                print("\nNy loppuu!")
                break

# Prints information about the servo
def status(id):
    read_id, comm_result, error = packetHandler.read1ByteTxRx(portHandler, id, 7)
    print("ID: " + str(read_id))
    msg, comm_result, error = packetHandler.read1ByteTxRx(portHandler, id, 70)
    print("HW Error: " + str(msg))
    msg, comm_result, error = packetHandler.read1ByteTxRx(portHandler, id, 32)
    print("VMax: " + str(msg))
    msg, comm_result, error = packetHandler.read1ByteTxRx(portHandler, id, 34)
    print("VMin: " + str(msg))
    print("\n")

# Nodding (unused)
def nod(id, pres_pos):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pres_pos + 800) #head down
    while True:
        dxl_after_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION) #read after position
        if abs(dxl_after_pos - goal_pos) < 10:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pres_pos - 1000) #move to max position

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pres_pos) #move to max position

def disable_torque(id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    return dxl_comm_result



def talker():
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def callback():
    pass

if __name__ == '__main__':
    portHandler = port_handler.PortHandler(DEVICENAME)
    packetHandler = packet_handler.PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # 1 second delay
    time.sleep(1)

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    init_servo(2)
    init_servo(3)

    zero_position_a = raise_neck(2)
    zero_position, comm_result, comm_error = packetHandler.read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION)

    scanmode(2, SHAKE_DIST, int(zero_position_a))
    scanmode(3, 300, zero_position)

    status(2)
    status(3)

    # Disable Dynamixel Torque
    disable_torque(2)
    disable_torque(3)

    portHandler.closePort()
    # rospy.spin()
