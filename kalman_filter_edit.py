import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import os
import sys
import time
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescImuStamped, VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from datetime import datetime
import pandas as pd
from time import strftime, gmtime

import csv

POSE_HEADER = ['s','ns','x','y','q.x','q.y','q.z','q.w']
VESC_HEADER = ['roll','pitch','yaw', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'x_q', 'y_q','z_q','w_q', 'roll_xi', 'pitch_yi', 'yaw_zi', 'velocity_y']
ACKR_HEADER = ['s','ns','V','delta']
ODOM_HEADER = ['velocity_x_actual', 'speedx', 'speedy', 'steeringangle', 'z_orientation', 'z', 'x', 'y', 'w']
SCAN_HEADER = ['s', 'ns', 'amin', 'amax', 'ai', 'ti', 'st', 'rmin', 'rmax']
CORE_HEADER = ['MC', 'CI', 'DC', 'RPM', 'ED', 'ER', 'VI', 'velocity_x_theoretical']
CALC_HEADER = ['velocity_y']
file_name1 = "core_values_log"
file1 = open('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/logs/' + file_name1 + '-wp-' + strftime('%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w')
file_name2 = "imu_values_log"
file2 = open('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/logs/' + file_name2 + '-wp-' + strftime('%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w')
file_name3 = "odom_values_log"
file3 = open('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/logs/' + file_name3 + '-wp-' + strftime('%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w')
file1.write(','.join(CORE_HEADER) + '\n')
file2.write(','.join(VESC_HEADER) + '\n')
file3.write(','.join(ODOM_HEADER) + '\n')




r = []
for i in range(1081):
    r += ['r' + str(i)]
SCAN_HEADER += r

class WallFollow(Node):
    """
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback,10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.imu_sub  = self.create_subscription(VescImuStamped, "/sensors/imu", self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.coresub = self.create_subscription(VescStateStamped, "/sensors/core", self.core_callback, 10)


        self.orient = 0
        self.file = None

        # TODO: store any necessary values you think you'll need
        #Desired distance to the wall
        self.prev_time = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.speed = 0
        self.velocity_x_actual = 0
        self.speed_y = 0
        self.MC     = 0
        self.CI     = 0
        self.DC     = 0
        self.RPM    = 0

        # Energy
        self.ED     = 0
        self.ER     = 0
        self.VI     = 0

        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        # Acceleration
        self.ax = 0
        self.ay = 0
        self.az = 0

        #velocity

        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Angular Frequency
        self.wx = 0
        self.wy = 0
        self.wz = 0

        # Quaternion
        self.x_q     = 0
        self.y_q     = 0
        self.z_q     = 0
        self.w_q     = 0

        # self.acc_time = []
        # self.acc_er = []
        # self.fig, self.ax = plt.subplots()

    def odom_callback(self, odom_msg):
        #global z,x,y,w,speed
        self.velocity_x_actual = odom_msg.twist.twist.linear.x
        self.speed = odom_msg.twist.twist.linear.x
        print("speeeedddd", self.speed)
        self.speed_y = odom_msg.twist.twist.linear.y
        print("speed_y_measured", self.speed_y)
        self.steering_angle = odom_msg.twist.twist.angular.z
        self.orientation = odom_msg.pose.pose.orientation.z


        self.z = odom_msg.pose.pose.orientation.z
        self.x = odom_msg.pose.pose.orientation.x
        self.y = odom_msg.pose.pose.orientation.y
        self.w = odom_msg.pose.pose.orientation.w
        file3.write('%f, %f, %f, %f,%f, %f, %f, %f, %f\n' % (self.velocity_x_actual, self.speed, self.speed_y, self.steering_angle,self.orientation, self.z,self.x,self.y,self.w ))

    def core_callback(self, core_msg):
        """
        Callback function for subscribing to VESC core sensor data.
        This funcion saves the current VESC State to a csv
        Args:
            core_msg (VescStateStamped): incoming message from subscribed topic
        """
        header = core_msg.header
        state  = core_msg.state

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # Motor Stats
        MC     = state.current_motor
        CI     = state.current_input
        DC     = state.duty_cycle
        RPM    = state.speed

        # Energy
        ED     = state.energy_drawn
        ER     = state.energy_regen
        VI     = state.voltage_input
        Gear_ratio = 11.85 #(ahmad data) # need to verify this using tachometer
        Tire_dia = 0.319 #m (ahmad data)
        Tire_dia = 0.11 #m (actual measured)

        wheel_speed = RPM / (Gear_ratio*60) # gear ratio should nbe 220 to meet with the actual speed which is strange
        #theoretical
        velocity_x_theoretical = (2*np.pi*RPM *(Tire_dia/2))/(Gear_ratio*2.5*60) # 2.5 is the final gear ratio (diff)



        

        

        # file = open(strftime('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w')

        file1.write('%f, %f, %f, %f,%f, %f, %f, %f\n' % (MC, CI, DC, RPM, ED, ER, VI, velocity_x_theoretical))

    def imu_callback(self, imu_msg):
        """
        Callback function for subscribing to VESC's imu data.
        This funcion saves the current IMU states to a csv file
        Args:
            imu_msg (VescImuStamped): incoming message from subscribed topic
        """
        header  = imu_msg.header
        imuData = imu_msg.imu

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # euler
        # global roll,pitch,yaw, ax,ay,az, wx, wy, wz,x_q,y_q,z_q,w_q
        roll  = imuData.ypr.x
        pitch = imuData.ypr.y
        yaw   = imuData.ypr.z

        # Acceleration
        ax = imuData.linear_acceleration.x
        ay = imuData.linear_acceleration.y
        az = imuData.linear_acceleration.z

        # Angular Frequency
        wx = imuData.angular_velocity.x
        wy = imuData.angular_velocity.y
        wz = imuData.angular_velocity.z

        # Quaternion
        x_q     = imuData.orientation.x
        y_q     = imuData.orientation.y
        z_q     = imuData.orientation.z
        w_q     = imuData.orientation.w



        w_i = w_q
        x_i = x_q
        y_i = y_q
        z_i = z_q
        t0 = +2.0 * (w_i * x_i + y_i * z_i)
        t1 = +1.0 - 2.0 * (x_i * x_i + y_i * y_i)
        roll_xi = math.atan2(t0, t1)

        t2 = +2.0 * (w_i * y_i - z_i * x_i)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_yi = math.asin(t2)

        t3 = +2.0 * (w_i * z_i + x_i * y_i)
        t4 = +1.0 - 2.0 * (y_i * y_i + z_i * z_i)
        yaw_zi = math.atan2(t3, t4)

        # print("orinetation",z) #printed
        print("angle z", yaw_zi)

        drive_msg = AckermannDriveStamped()
        r=self.speed
        print("speed", r)

        velocity_y = r * math.sin(yaw_zi)
        print("Velocity y", velocity_y)


  

        # file = open(strftime('/home/nvidia/f1tenth_ws/src/pure_pursuit/data/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w')

        file2.write('%f, %f, %f, %f,%f, %f, %f, %f, %f, %f, %f,%f,%f,%f,%f,%f, %f\n' % (roll,pitch,yaw, ax, ay, az, wx, wy, wz, x_q, y_q,z_q,w_q, roll_xi, pitch_yi, yaw_zi, velocity_y ))
    




    def calc_control():
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # s_angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        global drive_msg
        drive_msg = AckermannDriveStamped()

        # w = self.w
        # x = self.x
        # y = self.y
        # z = self.z
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll_x = math.atan2(t0, t1)

        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = math.asin(t2)

        # t3 = +2.0 * (w * z + x * y)
        # t4 = +1.0 - 2.0 * (y * y + z * z)
        # yaw_z = math.atan2(t3, t4)

        # # print("orinetation",z) #printed
        # print("angle z", yaw_z)



        # drive_msg = AckermannDriveStamped()


        # #Measured



        # #theoretical
        # velocity_y = self.speed * math.sin(yaw_z)



        # print("velocity_y", velocity_y)



    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        global message
        message = msg
        # self.pid_control() # TODO: actuate the car with PID
        global drive_msg
        drive_msg = AckermannDriveStamped()

        w = self.w
        x = self.x
        y = self.y
        z = self.z
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        print("angle z", yaw_z)

        drive_msg = AckermannDriveStamped()
        r=self.speed
        print("speed", r)

        velocity_y = r * math.sin(yaw_z)

        print("Velocity y", velocity_y)




        # drive_msg.drive.speed = 2.5
        # drive_msg.drive.steering_angle = 0.0
        # self.drive_pub.publish(drive_msg)
        # time.sleep(2)
        # drive_msg.drive.speed = 1.0
        # drive_msg.drive.steering_angle = 0.0
        # self.drive_pub.publish(drive_msg)
        # time.sleep(2)
        # drive_msg.drive.speed = 0.0
        # drive_msg.drive.steering_angle = 0.0
        # self.drive_pub.publish(drive_msg)
        # drive_msg.drive.speed = 0.5



        self.drive_pub.publish(drive_msg)



def main(args=None):

    rclpy.init(args=args)
    print("WallFollow Initialized")
    # stateEstimation_node = stateEstimation(dt_string)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    if wall_follow_node.file:  # Check if the file attribute has a value before attempting to close it
        wall_follow_node.file.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
