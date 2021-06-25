#!/usr/bin/env python
import socket

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

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
    #t_=rospy.Time.now()
    return t_

def getAngle(sensor, bgz, dt, fil_angle):
    
    GyrToDegPerSec = 131.;
    GyZ = sensor.read_i2c_word(0x47)
    
    #t_now = int(time.time()*1000.0)
    #dt = (t_now-t_prev)/1000.0

    gyro_z = (GyZ-bgz)/GyrToDegPerSec
    tmp_angle_z = fil_angle.z+gyro_z*dt
    fil_angle.z = tmp_angle_z

    return fil_angle.z

## comm - socket
ip = "192.168.0.154"
sock_port = 9001

sock_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_server.bind((ip, sock_port))

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


def commandThread():
    global command
    global ser
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
        
    '''
    while True:
        command = sys.stdin.readline()[:-1] # [0]
        ser.write(command.encode())
    '''
    sock_server.close()
    sock_client.close()

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
    obj_dist = 30
    
    while True:
        for i in range(3):                
            GPIO.output(FRONT_t[i], False)
            time.sleep(0.5)
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
                command = 'x'
                ser.write(command.encode())
            #print(distance)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LEFT, GPIO.IN)
GPIO.setup(RIGHT, GPIO.IN)

for i in range(3):        
    GPIO.setup(FRONT_t[i], GPIO.OUT)
    GPIO.setup(FRONT_e[i], GPIO.IN)

rospy.init_node('odometry_publisher')

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

vx = 0.0
vy = 0.0
vth = 0.0

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
    
    """
    # original
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt    # + vy * cos(pi/2-th)
    delta_y = (vx * sin(th) + vy * cos(th)) * dt    # + vy * sin(pi/2-th)
    
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th
    """
    
    dt = (current_time - last_time).to_sec()
    
    # get distance 
    if command=='w':
        avg_cnt = (cnt_r+cnt_l-2)/2.
        delta_dist = (avg_cnt-prev_cnt)/40.*wheel
        prev_cnt=avg_cnt
    else:
        cnt_r=0
        cnt_l=0
        prev_r=-1
        prev_l=-1
        prev_cnt=0
        delta_dist=0
    
    now_th = getAngle(sensor,BGZ, dt, fil_angle)
    th = (-1)*now_th*math.pi/180
    #print(delta_dist)
    total_dist+=delta_dist
    #print(total_dist,cnt_l, cnt_r)
    delta_x = delta_dist*math.cos(th)
    delta_y = delta_dist*math.sin(th)
    
    delta_th = (now_th-prev_th)*math.pi/180
    
    vx=delta_x/dt
    vy=delta_y/dt
    vth = delta_th/dt
    
    x += delta_x
    y += delta_y
    #print(delta_x,delta_y,delta_dist, th)
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

    last_time = current_time
    #prev_th = now_th
    
    r.sleep()
