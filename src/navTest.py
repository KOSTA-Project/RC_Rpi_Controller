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

# functions for mpu calc
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

def getAngle(sensor, bgz, t_prev, fil_angle):
    
    t_now = int(time.time()*1000.0)
    dt = (t_now-t_prev)/1000.0
    
    GyrToDegPerSec = 131.;
    GyZ = sensor.read_i2c_word(0x47)
    
    gyro_z = (GyZ-bgz)/GyrToDegPerSec
    tmp_angle_z = fil_angle.z+gyro_z*dt
    fil_angle.z = tmp_angle_z
    
    return fil_angle.z,t_now

# for move dist calc
command = ''
cnt_r =0 ; cnt_l=0
prev_r=-1; prev_l=-1

### for motor op
port = "/dev/ttyACM0"
ser = serial.Serial(port, 115200)
# encoder 
LEFT = 7
RIGHT = 22
wheel=21.2/100.  # meter

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LEFT, GPIO.IN)
GPIO.setup(RIGHT, GPIO.IN)

sensor = mpu6050(0x68)
bus = smbus.SMBus(1)
Device_Address = 0x68

BGZ = calibAccelGyro(sensor)
fil_angle = Fil_angle()

cur_th = 0.0

# read encoder
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


# start - variables init
rospy.init_node('nav_node')

last_time_sub = -1

isnav = 0
init=0
# msg about move for path planning

vx_sub=0
vy_sub=0
vth_sub=0


def subscribe_cmd_vel(data):
    global init
    global isnav
    global last_time_sub
    global vx_sub
    global vy_sub
    global vth_sub
    
    if init==0:
	init=1
	return
    
    if isnav==1: return
    isnav=1
    # when first arrived 
    if last_time_sub==-1:
	last_time_sub = rospy.Time.now()
	
	vx_sub= data.linear.x
	vy_sub= data.linear.y
	vth_sub= data.angular.z
	
	isnav = 0
	return
    
    current_time_sub = rospy.Time.now()
    dt = (current_time_sub - last_time_sub).to_sec()
    last_time_sub = -1
    
    # vx,vy,vth original position
    
    move(vx_sub,vy_sub,vth_sub,dt)

def move(vx,vy,vth,dt):
	
	global command
	global ser
	global cnt_r
	global cnt_l
	global sensor
	global BGZ
	global fil_angle
	global isnav
	
	limit = 1*math.pi/180
	limit_dist = 1e-3
	print(rospy.Time.now().to_sec(), dt)
	# idle
	delta_th = vth*dt
	delta_x = vx*dt
	delta_y = vy*dt
	dist = math.sqrt(delta_x*delta_x + delta_y*delta_y) #target
	
	#print("target {0:}\t{1:}\t{2:}\t{3:}\n".format(delta_x,delta_y, delta_th,dt))
		
	# real move
	dist_now =0
	t_p = initDt()
	
	now_th, t_p = getAngle(sensor,BGZ, t_p, fil_angle)
	th = (-1)*now_th*math.pi/180
	
	target = delta_th+th
	
	#roatate
	if vth!=0.0:
	    
	    #target %= (2*math.pi)
	    #print("vth op: {0:}\t{1:}\t{2:}\t{3:}\n".format(delta_th,th,target,dt))
	    
	    if target < th: command = 'd'
	    else: command='a'
	    
	    if th-target > 0:
		t_p = initDt()
		ser.write(command.encode())
		while th-target>limit:
		    now_th,t_p = getAngle(sensor,BGZ, t_p, fil_angle)
		    th = (-1)*now_th*math.pi/180
		    
		    #print(delta_th,th,target, command) #th --> now, angle(th)
		    time.sleep(0.005)
	    else:
		t_p = initDt()
		ser.write(command.encode())
		while target-th>limit:
		    now_th,t_p = getAngle(sensor,BGZ, t_p, fil_angle)
		    th = (-1)*now_th*math.pi/180
		    #print(delta_th,th,target, command)
		    time.sleep(0.005)
	    command = 'x'
	    ser.write(command.encode())
	# go straight
	if dist>limit_dist:
	    #dist += dist_now
	    command = 'w'
	    ser.write(command.encode())
	    while dist_now < dist:
		    avg_cnt = (cnt_r+cnt_l-2)/2.
		    dist_now = (avg_cnt)/40.*wheel
		    #print(dist_now,dist,command)
		    time.sleep(0.005)
	    command = 'x'
	    ser.write(command.encode())
	
	print("message complete: target-{0:}\t{1:}\tcur-{2:}\t{3:}\n".format(dist, target, dist_now, th))
	
	#last_time_sub = rospy.Time().now()
	isnav = 0

def sendOdom():
    global fil_angle
    global sensor
    global BGZ
    global dt
    global cnt_r
    global cnt_l
    global prev_r
    global prev_l
    
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    prev_th =0
    prev_cnt =0.0

    # dt, th, delta_dist
    r = rospy.Rate(80)
    t_p = initDt()
    while not rospy.is_shutdown() :
	#print("odom while")
	current_time = rospy.Time.now()
	dt = (current_time - last_time).to_sec()
	
	now_th, t_p = getAngle(sensor,BGZ, t_p, fil_angle)
	last_time = current_time
	
	th = (-1)*now_th*math.pi/180
	delta_th = th-prev_th
	#delta_th = th	#-prev_th
	
	'''
	avg_cnt = (cnt_r+cnt_l-2)/2.
	delta_dist = (avg_cnt-prev_cnt)/40.*wheel
	prev_cnt=avg_cnt
	'''
	# get delta_dist
	if command=='w' or command=='s':
	    avg_cnt = (cnt_r+cnt_l-2)/2.
	    delta_dist= (avg_cnt-prev_cnt)/40.*wheel
	    prev_cnt=avg_cnt
	else:
	    cnt_r=0
	    cnt_l=0
	    prev_r=-1
	    prev_l=-1
	    prev_cnt=0
	    delta_dist=0
	
	
	delta_x = delta_dist*math.cos(th)
	delta_y = delta_dist*math.sin(th)
	
	vx = delta_x/dt
	vy = delta_y/dt
	vth = delta_th/dt
	
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
	prev_th = th
	
	r.sleep()

# Run Thread
distThread = threading.Thread(target=distanceThread)
distThread.daemon = True
distThread.start()

odomThread = threading.Thread(target=sendOdom)
odomThread.daemon = True
odomThread.start()

'''
while True:
    command = sys.stdin.readline()[:-1] # [0]
    ser.write(command.encode())
'''

rospy.Subscriber('cmd_vel',Twist,subscribe_cmd_vel)

print(" Reading Data of Gyroscope and Accelerometer")
rospy.spin()
