#!/usr/bin/env python

import time
import serial
import string
import math

import rospy
from sensor_msgs.msg import Imu, MagneticField

IMU_FRAME = 'imu_link'
GRAD2RAD = 3.141592654/180.0
RAD2GRAD = 180.0/3.141592654

G = 9.794 #Shanghai
SA = 0.001195557
SG = 0.0304878
SM = 0.9174312
    
class ArduIMUROS(object):

    def __init__(self):
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size = 0) 
        self.mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size = 0)      
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", "115200"))

        self.imuMsg = Imu()
        self.imuMsg.orientation_covariance = [0.0025 , 0 , 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self.imuMsg.angular_velocity_covariance = [0.02, 0 , 0, 0 , 0.02, 0, 0 , 0 , 0.02]
        self.imuMsg.linear_acceleration_covariance = [0.04, 0 , 0, 0 , 0.04, 0, 0 , 0 , 0.04]

        self.magMsg = MagneticField()
        self.magMsg.magnetic_field_covariance = [2.5e-5, 0 , 0, 0 , 2.5e-5, 0, 0 , 0 , 2.5e-5]

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []
    
        self.gyro_x_offset = 0.0
        self.gyro_y_offset = 0.0
        self.gyro_z_offset = 0.0

        self.start()

        i = 0
        while i < 300:
            try:
                line = self._Serial.readline()
            except Exception:
                print 'Calibration ERROR!'

            words = string.split(line,",")
            if len(words) == 10:
                try:  
                    self.gyro_x.append(float(words[4]))
                    self.gyro_y.append(float(words[5]))
                    self.gyro_z.append(float(words[6]))
                except Exception as e:
                    print 'value error', e
            i += 1
        self.gyro_x_offset = sum(self.gyro_x)/len(self.gyro_x)
        self.gyro_y_offset = sum(self.gyro_y)/len(self.gyro_y)
        self.gyro_z_offset = sum(self.gyro_z)/len(self.gyro_z)

        while not rospy.is_shutdown():
            self.process()
        self.stop()

    def start(self):
        self._Serial = serial.Serial(self.port, self.baud, timeout=1)
        self._Serial.flushInput()
        self._Serial.flushOutput()

    def stop(self):
        rospy.loginfo("Stopping serial gateway")
        time.sleep(.1)
        self._Serial.close()

    def process(self):
        try:
            line = self._Serial.readline()
        except Exception:
            print 'Interrupted'
            return

        words = string.split(line,",")
        if len(words) == 10:
            try:   
                self.imuMsg.linear_acceleration.x = (float(words[1]) - (417))*SA
                self.imuMsg.linear_acceleration.y = (float(words[2]) - (-132))*SA
                self.imuMsg.linear_acceleration.z = (float(words[3]) - (-2000))*SA

                self.imuMsg.angular_velocity.x = (float(words[4]) - self.gyro_x_offset)*SG*GRAD2RAD
                self.imuMsg.angular_velocity.y = (float(words[5]) - self.gyro_y_offset)*SG*GRAD2RAD
                self.imuMsg.angular_velocity.z = (float(words[6]) - self.gyro_z_offset)*SG*GRAD2RAD
                if (-0.002 < self.imuMsg.angular_velocity.z < 0.002):
                    self.imuMsg.angular_velocity.z = 0.000

                self.magMsg.magnetic_field.y = -(float(words[7]) - 0)*SM*(1e-7)
                self.magMsg.magnetic_field.x = -(float(words[8]) - 0)*SM*(1e-7)
                self.magMsg.magnetic_field.z = (float(words[9]) - 0)*SM*(1e-7)

            except Exception as e:
                print 'value error', e
        
        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = IMU_FRAME
        self.imu_pub.publish(self.imuMsg)

        self.magMsg.header.stamp= rospy.Time.now()
        self.magMsg.header.frame_id = IMU_FRAME
        self.mag_pub.publish(self.magMsg)
       
if __name__ == '__main__': 
    rospy.init_node('arduimu_ros') 
    ArduIMUROS()
