# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
# Type of trajectory
PARABOLA = "PARABOLA"; SIGMOID = "SIGMOID"
from math import exp
import numpy as np


class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0], trajectory=PARABOLA):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(trajectory)


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, trajectory):
        trajectory_list = []
        step = 0.01
        if trajectory == PARABOLA:
            x_range = np.arange(0, 1.5 + step, step)
            for x in x_range:
                y = x ** 2
                trajectory_list.append([x, y])

        elif trajectory == SIGMOID:
            x_range = np.arange(0, 2.5 + step, step)
            for x in x_range:
                y = 2 / (1 + exp(-2 * x)) - 1
                trajectory_list.append([x, y])

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return trajectory_list

