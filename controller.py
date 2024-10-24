import numpy as np


from pid import PID_ctrl, P, PD, PI, PID
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, control_type=P, file_name=None):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(control_type, klp, klv, kli, filename_=f"linear{file_name}.csv")
        self.PID_angular=PID_ctrl(control_type, kap, kav, kai, filename_=f"angular{file_name}.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        # SIM VALUES
        linear_vel = 0.22 if linear_vel > 0.22 else linear_vel
        angular_vel = 2.84 if angular_vel > 2.84 else angular_vel
        # REAL ROBOT (linear vel 0.46 if safe mode off)
        # linear_vel = 0.31 if linear_vel > 0.31 else linear_vel
        # angular_vel = 1.90 if angular_vel > 1.90 else angular_vel
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, control_type=P, file_name=None):
        
        super().__init__(klp, klv, kli, kap, kav, kai, control_type, file_name)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity
        # SIM VALUES
        linear_vel = 0.22 if linear_vel > 0.22 else linear_vel
        angular_vel = 2.84 if angular_vel > 2.84 else angular_vel
        # REAL ROBOT (linear vel 0.46 if safe mode off)
        # linear_vel = 0.31 if linear_vel > 0.31 else linear_vel
        # angular_vel = 1.90 if angular_vel > 1.90 else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
