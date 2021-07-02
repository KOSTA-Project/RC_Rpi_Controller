import smbus
from time import sleep
import time
import math
from mpu6050 import mpu6050


class Gyro_angle:
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0
        
class Fil_angle:
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0

def calibAccelGyro(sensor):

    sumGx = 0; sumGy = 0; sumGz = 0
    # Ax, Ay, Az, Gx, Gy, Gz = readAccelGyro()
    for _ in range(10):
        Gz= sensor.read_i2c_word(0x47)
        sumGz += Gz
        sleep(0.01)
    baseGz = sumGz / 10
    
    return baseGz


def initDt():
    t_= int(time.time()*1000.0)
    return t_

def getAngle(sensor, bgz,t_prev, fil_angle):
    
    alpha=0.96
    GyrToDegPerSec = 131.;
    GyZ = sensor.read_i2c_word(0x47)
    t_now = int(time.time()*1000.0)
    dt = (t_now-t_prev)/1000.0

    gyro_z = (GyZ-bgz)/GyrToDegPerSec


    tmp_angle_z = fil_angle.z+gyro_z*dt

    fil_angle.z = tmp_angle_z
    

    return fil_angle.z, t_now

sensor = mpu6050(0x68)
bus = smbus.SMBus(1)
Device_Address = 0x68

print(" Reading Data of Gyroscope and Accelerometer")

BGZ = calibAccelGyro(sensor)
t_p = initDt()
fil_angle = Fil_angle()

while True:
    
    fz, t_p= getAngle(sensor, BGZ, t_p, fil_angle)
    print("{0:.2f}".format(fz))
    sleep(0.1)

bus.close()
