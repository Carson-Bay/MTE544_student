# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion, M_PI
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
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
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        
        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        if not self.imu_initialized:
            self.imu_initialized = True

        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        imu_ang_vel = imu_msg.angular_velocity
        imu_lin_acc = imu_msg.linear_acceleration

        acc_x = imu_lin_acc.x
        acc_y = imu_lin_acc.y
        angular_z = imu_ang_vel.z

        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])
        
    def odom_callback(self, odom_msg: Odometry):
        if not self.odom_initialized:
            self.odom_initialized = True

        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        
        odom_orientation = odom_msg.pose.pose.orientation

        # odom_pos contains x, y, and z 
        odom_pos = odom_msg.pose.pose.position

        x = odom_pos.x
        y = odom_pos.y
        # TODO: Haven't seen the lecture on quaternions yet so dont know how to find the theta yet
        # I can probably come back to this when I catch up on the lectures hopefully today lol
        th = odom_orientation.w
        th = odom_orientation.y
    
        self.odom_logger.log_values([x, y, th, timestamp])
                

    def laser_callback(self, laser_msg: LaserScan):
        if not self.laser_initialized:
            self.laser_initialized = True

        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        max_range = laser_msg.range_max
        min_range = laser_msg.range_min
        angle = laser_msg.angle_min
        angle_max = laser_msg.angle_max
        angle_increment = laser_msg.angle_increment
        
        for measured_range in laser_msg.ranges:
            if angle > angle_max:
                # Not sure what scenarios will trigger this
                print(f"angle {angle} > angle_max {angle_max}")

            # NOTE: I'm assuming the first range measurement corresponds to angle_min
            # and each following measurement corresponds to prior angle + increment
            self.laser_logger.log_values([measured_range, angle, timestamp])
            angle += angle_increment
            # NOTE: I think all data outside of the range of the sensor are broadcast as .inf, not sure if we want to keep


        # laser_ranges = laser_msg.ranges

        # for range in laser_ranges:
        #     if laser_ranges[range] < min_range:
        #         # discard data
        #         pass
        #     elif laser_ranges[range] > max_range:
        #         # discard data
        #         pass
                
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
        # TODO: Not actually sure whether these should be constants or some input parameter or smth
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
        # Spiral radius increases based on timer cycle time
        self.radius_ += 0.01 # [m]

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
