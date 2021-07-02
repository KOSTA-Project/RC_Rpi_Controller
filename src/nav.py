#!/usr/bin/env python
import socket

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Polygon

import RPi.GPIO as GPIO
from time import sleep

import threading
import sys
import serial

import time
from mpu6050 import mpu6050
import smbus

class Fil_angle:
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0

def calibAccelGyro(sensor):
    sumGz = 0
    for _ in range(10):
        Gz= sensor.read_i2c_word(0x47)
        sumGz += Gz
        sleep(0.01)
    baseGz = sumGz / 10
    
    return baseGz


def initDt():
    t_= int(time.time()*1000.0)
    return t_

def getAngle(sensor, bgz, dt, fil_angle):
    
    GyrToDegPerSec = 131.;
    GyZ = sensor.read_i2c_word(0x47)
    
    gyro_z = (GyZ-bgz)/GyrToDegPerSec
    tmp_angle_z = fil_angle.z+gyro_z*dt
    fil_angle.z = tmp_angle_z

    return fil_angle.z

## comm - socket
ip = "192.168.0.21"
sock_port = 9001

#sock_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#sock_server.bind((ip, sock_port))

## for dist - thread
command = ''
cnt_r =0 ; cnt_l=0
prev_r=-1; prev_l=-1

port = "/dev/ttyACM0"
ser = serial.Serial(port, 115200)

# get distance 
LEFT = 7
RIGHT = 22
wheel=21.2/100.  # meter

# ultra sonic
FRONT_t = [11,13,16]
FRONT_e = [12,15,18]

vx = 0.0
vy = 0.0
vth = 0.0

ddist=0.0
dth = 0.0
dt = 0

rospy.init_node('nav_node')

current_time_sub = rospy.Time.now()
last_time_sub = rospy.Time.now()

def subscribe_cmd_vel(data):
	
	global current_time_sub
	global last_time_sub
	
	global vx
	global vy
	global vth
	'''
	current_time_sub = rospy.Time.now()
	dt = (current_time_sub - last_time_sub).to_sec()
	last_time_sub = current_time_sub
	'''
	# received values
	vx= data.linear.x
	vy= data.linear.y
	vth= data.angular.z
	
	

def commandThread():
    global command
    global ser
    
    while True:
        command = sys.stdin.readline()[:-1] # [0]
        ser.write(command.encode())
    
    '''
    global sock_server
    op = 'x'
    
    while True:
        sock_server.listen(5)
        sock_client, addr = sock_server.accept()
        command = sock_client.recv(1024)
        if len(command)==0: continue
        command = command[0]
        if op == command: continue
        op = command
        print(command.decode())
        ser.write(command.encode())
    
    sock_server.close()
    sock_client.close()
    '''
    

def distanceThread():
    global cnt_r
    global cnt_l
    global command
    global prev_r
    global prev_l
    
    while True:
        if command=='w':
            rr = GPIO.input(RIGHT)
            ll = GPIO.input(LEFT)
            if prev_r != rr:
                cnt_r+=1
                prev_r = rr
            if prev_l != ll:
                cnt_l+=1
                prev_l=ll
        time.sleep(0.003)

def usThread():
    global command
    global ser
    global buz
    obj_dist = 15
    GPIO.output(buz,False)
    while True:
        for i in range(3):                
            GPIO.output(FRONT_t[i], False)
            time.sleep(0.1)
            GPIO.output(FRONT_t[i], True)
            time.sleep(0.00001)
            GPIO.output(FRONT_t[i], False)
            while GPIO.input(FRONT_e[i]) == 0:
                pulse_start = time.time()
            while GPIO.input(FRONT_e[i]) == 1:
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17000
            distance = round(distance, 2)
            if distance < obj_dist:
		GPIO.output(buz,True)
                command = 'x'
                ser.write(command.encode())
		time.sleep(0.001)
	    
            #print(i, distance)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LEFT, GPIO.IN)
GPIO.setup(RIGHT, GPIO.IN)
GPIO.setup(buz, GPIO.OUT)

for i in range(3):        
    GPIO.setup(FRONT_t[i], GPIO.OUT)
    GPIO.setup(FRONT_e[i], GPIO.IN)


rospy.Subscriber('cmd_vel',Twist,subscribe_cmd_vel)

sensor = mpu6050(0x68)
bus = smbus.SMBus(1)
Device_Address = 0x68

print(" Reading Data of Gyroscope and Accelerometer")

BGZ = calibAccelGyro(sensor)
fil_angle = Fil_angle()

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

prev_th=0.0


# Run Thread
distThread = threading.Thread(target=distanceThread)
commThread = threading.Thread(target=commandThread)
usThread = threading.Thread(target=usThread)

distThread.daemon = True
commThread.daemon = True
usThread.daemon = True

distThread.start()
commThread.start()
usThread.start()

###

prev_cnt=0
total_dist=0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)
while not rospy.is_shutdown():
    
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    last_time = current_time
    
    dx = vx*dt
    dy = vy*dt
    dth = vth*dt
    ddist = math.sqrt(dx*dx+dy*dy)
    
    th += dth 	# total angle (=current angle)
    
    #### 
    delta_x = ddist*math.cos(th)
    delta_y = ddist*math.sin(th)
    
    vx=delta_x/dt
    vy=delta_y/dt
    vth = dth/dt
    
    x += delta_x
    y += delta_y
    
    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    # publish the message
    odom_pub.publish(odom)

    #last_time = current_time
    #prev_th = now_th
    
    r.sleep()
