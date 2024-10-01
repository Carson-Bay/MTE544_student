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
        # TODO: Temp variable for spiral motion, will probably change or fix later
        self.spiral_forward_vel = 0.01
        
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
        
        ...
        
        # ENCODER subscription

        ...
        
        # LaserScan subscription 
        
        ...
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        ...    # log imu msgs
        
    def odom_callback(self, odom_msg: Odometry):
        
        ... # log odom msgs
                
    def laser_callback(self, laser_msg: LaserScan):
        
        ... # log laser msgs with position msg at that time
                
    def timer_callback(self):
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            # TODO: Temporary change for testing, delete the pass and uncomment the return
            pass
            # return
        
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
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        # TODO: Not actually sure whether these should be constants or some input parameter or smth
        dir = -1 # 1 for CCW, -1 for CW
        turn_radius = 1.0 # [m] TODO: Not actually sure what the units are here
        forward_vel = 1.0 # [m/s] TODO: Not actually sure what the units are here

        msg=Twist()
        msg.linear.x = forward_vel
        msg.angular.z = dir * forward_vel / turn_radius

        return msg

    def make_spiral_twist(self):
        # TODO really sketch will return to this
        dir = -1 # 1 for CCW, -1 for CW
        start_radius = 1.0 # [m] TODO: Not actually sure what the units are here
        forward_vel = 1.0 # [m/s] TODO: Not actually sure what the units are here
        angular_vel = dir * forward_vel / start_radius

        msg=Twist()
        msg.linear.x = self.spiral_forward_vel
        msg.angular.z = angular_vel
        # msg.linear.y doesn't seem to do anything so either adjust angular or forward vel to create spiral?
        # msg.angular.y = 2 * M_PI / angular_vel * 0.1
        self.spiral_forward_vel += 0.01
        
        return msg
    
    def make_acc_line_twist(self):
        forward_vel = 1.0 # [m/s]
        
        msg=Twist()
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
