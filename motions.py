# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_yaw_from_quaternion
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type = motion_type
        
        self.radius_ = 0.0
        
        self.successful_init = False
        self.imu_initialized = False
        self.odom_initialized = False
        self.laser_initialized = False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
                
        # loggers
        self.imu_logger = Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger = Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y", "quat_z", "quat_w", "th", "stamp"])
        self.laser_logger = Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        # are we supposed to use the qos for the vel_publisher as well?
        qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        
        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)
        
        self.timer_period_s = 0.1
        self.create_timer(self.timer_period_s, self.timer_callback)

    def imu_callback(self, imu_msg: Imu):
        if not self.imu_initialized:
            self.imu_initialized = True

        imu_ang_vel = imu_msg.angular_velocity
        imu_lin_acc = imu_msg.linear_acceleration

        acc_x = imu_lin_acc.x
        acc_y = imu_lin_acc.y
        angular_z = imu_ang_vel.z
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])
        
    def odom_callback(self, odom_msg: Odometry):
        if not self.odom_initialized:
            self.odom_initialized = True

        # odom_orientation contains quaternion values
        odom_orientation = odom_msg.pose.pose.orientation
        # odom_pos contains x, y, and z 
        odom_pos = odom_msg.pose.pose.position

        x = odom_pos.x
        y = odom_pos.y
        quat_z = odom_orientation.z
        quat_w = odom_orientation.w
        th = euler_yaw_from_quaternion(odom_orientation)
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        self.odom_logger.log_values([x, y, quat_z, quat_w, th, timestamp])

    def laser_callback(self, laser_msg: LaserScan):
        """
        NOTE: Laser mounted rigidly to robot (laser frame rotates with robot), so need to consider
        robot orientation and position in addition to sensor angles / range measurement if trying to map
        out obstacles or something
        """
        if not self.laser_initialized:
            self.laser_initialized = True

        max_range = laser_msg.range_max
        min_range = laser_msg.range_min

        angle = laser_msg.angle_min
        angle_max = laser_msg.angle_max
        angle_increment = laser_msg.angle_increment
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        for measured_range in laser_msg.ranges:
            if angle > angle_max:
                # Not sure what scenarios will trigger this, add an asterisk to check
                print(f"angle {angle} > angle_max {angle_max}")
                angle = f"{angle}*"

            """
            NOTE: I'm assuming the first range measurement corresponds to angle_min
            and each following measurement corresponds to prior angle + increment
            NOTE: I think all data outside of the range of the sensor are broadcast as .inf,
            not sure how we want to process that
            """
            # if min_range <= measured_range <= max_range:
            self.laser_logger.log_values([measured_range, angle, timestamp])

            angle += angle_increment

    def timer_callback(self):
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    def make_circular_twist(self):
        dir = -1 # 1 for CCW, -1 for CW
        turn_radius = 1.0 # [m]
        forward_vel = 1.0 # [m/s]

        msg = Twist()
        msg.linear.x = forward_vel
        msg.angular.z = dir * forward_vel / turn_radius

        return msg

    def make_spiral_twist(self):
        dir = -1 # 1 for CCW, -1 for CW
        forward_vel = 1.0 # [m/s]
        radius_increase_per_second = 0.1 # [m]
        # Spiral radius increases based on timer cycle time
        self.radius_ += radius_increase_per_second * self.timer_period_s

        msg = Twist()
        msg.linear.x = forward_vel
        msg.angular.z = dir * forward_vel / self.radius_
        
        return msg
    
    def make_acc_line_twist(self):
        forward_vel = 1.0 # [m/s]
        
        msg = Twist()
        msg.linear.x = forward_vel

        return msg

import argparse

if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
