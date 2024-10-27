import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np
from math import exp



def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))


    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].scatter(values[0][0], values[0][1], c="green") # start
    axes[0].scatter(values[-1][0], values[-1][1], c="red") # end

    # # parabola ref plot
    # trajectory_list = [[ (x/25.0) ,(x/25.0)**2] for x in range(0,50)]
    # traj_list = np.asarray(trajectory_list)
    # axes[0].scatter(traj_list[:,0], traj_list[:,1], s=5, c="green")

    # # sigmoid ref plot
    # x_range = np.linspace(0, 2.5, 20)
    # # x_range = np.arange(0, 2.5 + step, step)
    # for x in x_range:
    #     y = 2 / (1 + exp(-2 * x)) - 1
    #     trajectory_list.append([x, y])
    # traj_list = np.asarray(trajectory_list)
    # axes[0].scatter(traj_list[:,0], traj_list[:,1], s=5, c="green")

    axes[0].set_title("state space")
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].grid()

    plt.show()

def plot_linear_angular_errors(linear_filename, angular_filename, control_type, trajectory):

    headers, values=FileReader(linear_filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    fig, axes = plt.subplots(1,2, figsize=(14,6))
    
    axes[0].set_title(f"Linear Errors for {control_type} {trajectory}")
    # for i in range(0, len(headers) - 1):
    #     axes[0].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    axes[0].plot(time_list, [lin[0] for lin in values], label= headers[0]+ " linear [m]")
    axes[0].plot(time_list, [lin[1] for lin in values], label= headers[1]+ " linear [m/s]")
    axes[0].plot(time_list, [lin[2] for lin in values], label= headers[2]+ " linear [m*s]")

    axes[0].legend()
    axes[0].grid()
    axes[0].set_xlabel("Error Value")
    axes[0].set_ylabel("Time [s]")

    headers, values=FileReader(angular_filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
    
    axes[1].set_title(f"Angular Errors for {control_type} {trajectory}")
    # for i in range(0, len(headers) - 1):
    #     axes[0].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " angular")
    axes[1].plot(time_list, [lin[0] for lin in values], label= headers[0]+ " angular [rad]")
    axes[1].plot(time_list, [lin[1] for lin in values], label= headers[1]+ " angular [rad/s]")
    axes[1].plot(time_list, [lin[2] for lin in values], label= headers[2]+ " angular [rad*s]")

    axes[1].legend()
    axes[1].grid()
    axes[1].set_xlabel("Error Value")
    axes[1].set_ylabel("Time [s]")
    

def plot(control_type: str, trajectory: str):
    
    headers, values=FileReader(f"robot_pose_{control_type}_{trajectory}.csv").read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    fig, axes = plt.subplots(1,2, figsize=(14,6))

    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values], label="Real Trajectory")
    axes[0].scatter(values[0][0], values[0][1], c="green") # start
    axes[0].scatter(values[-1][0], values[-1][1], c="red") # end

    axes[0].set_title(f"State Space for {control_type} {trajectory}")
    axes[0].grid()
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend()

    
    axes[1].set_title(f"States of x, y, theta for {control_type} {trajectory}")
    # for i in range(0, len(headers) - 1):
    #     axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])
    axes[1].plot(time_list, [lin[0] for lin in values], label= headers[0] + " [m]")
    axes[1].plot(time_list, [lin[1] for lin in values], label= headers[1] + " [m]")
    axes[1].plot(time_list, [lin[2] for lin in values], label= headers[2] + " [rad]")
    

    axes[1].legend()
    axes[1].grid()
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Value")

    plot_linear_angular_errors(f"linear_{control_type}_{trajectory}.csv", f"angular_{control_type}_{trajectory}.csv", control_type, trajectory)

    return axes

def plot_parabola(control_type: str):
    axes = plot(control_type, "PARABOLA")
    trajectory_list = []
    x_range = np.linspace(0, 1.5, 20)
    for x in x_range:
        y = x ** 2
        trajectory_list.append([x, y])
    trajectory_list = np.asarray(trajectory_list)

    axes[0].scatter(trajectory_list[:,0], trajectory_list[:,1], s=5, c="green", label="Planner Trajectory")
    axes[0].legend()

def plot_sigmoid(control_type: str):
    axes = plot(control_type, "SIGMOID")
    trajectory_list = []
    x_range = np.linspace(0, 2.5, 20)
    for x in x_range:
        y = 2 / (1 + exp(-2 * x)) - 1
        trajectory_list.append([x, y])
    traj_list = np.asarray(trajectory_list)
    axes[0].scatter(traj_list[:,0], traj_list[:,1], s=5, c="green", label="Planner Trajectory")
    axes[0].legend()





import argparse

if __name__=="__main__":

    # parser = argparse.ArgumentParser(description='Process some files.')
    # parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    # args = parser.parse_args()
    
    # print("plotting the files", args.files)

    # filenames=args.files
    # for filename in filenames:
    #     plot_errors(filename)

    plot("P", "POINT")
    plot("PID", "POINT")

    plot_parabola("PID")
    plot_sigmoid("PID")
    plt.show()



