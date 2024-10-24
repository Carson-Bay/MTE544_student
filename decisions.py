# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController, P, PD, PI, PID

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(
        self,
        publisher_msg,
        publishing_topic,
        qos_publisher, 
        goalPoint=[-1, -1],
        rate=10,
        motion_type=POINT_PLANNER,
        control_type=PID,
    ):
        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
        if motion_type == POINT_PLANNER:
            self.controller=controller(
                klp=0.2, # linear P
                klv=0.5, # linear D
                kli=0.2, # linear I
                kap=0.8, # angular P
                kav=0.6, # angular D
                kai=0.2, # angular I
                control_type=control_type,
            )
            self.planner=planner(POINT_PLANNER)    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(
                klp=0.2, # linear P
                klv=0.5, # linear D
                kli=0.2, # linear I
                kap=0.8, # angular P
                kav=0.6, # angular D
                kai=0.2, # angular I
                control_type=control_type,
            )
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor, control_type=control_type)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        error_threshold = 0.01 # dummy error threshold value for now
        if type(self.goal) == list:
            # trajectory_planner return type [[x1,y1], ..., [xn,yn]]
            reached_goal = calculate_linear_error(self.localizer.pose, self.goal[-1]) < error_threshold
        else:
            # point_planner return type (x, y)
            reached_goal = calculate_linear_error(self.localizer.pose, self.goal) < error_threshold
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            raise SystemExit
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate

        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    qos = QoSProfile(reliability=1, durability=2, history=1, depth=10)
    

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", qos, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", qos, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        

    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
