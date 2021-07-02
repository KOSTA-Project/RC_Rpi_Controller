#!/usr/bin/env python
import os, sys
import rospy

rospy.init_node("cam")
str = 'mjpg_streamer -i "input_raspicam.so -vf -hf" -o "output_http.so -p 8090 -w /usr/local/share/mjpg-streamer/www/"'
os.system(str)
