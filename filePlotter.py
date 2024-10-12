# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import math
import numpy as np

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()

def imu_plotter(filename):
    # extracting data from the file, same as given example in plot_errors
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
    

    # setting up filtering variables, filter strength determines window size
    filter_strength = 10
    filtered_values = []
    data = []
    sum_values = []

    # mean value filtering the data
    for i in range(0, len(headers) - 1):
        # data is a list of each individual accelerations
        # i.e. first iteration it is all x values, second iteration is all y values, etc
        data = [lin[i] for lin in values]

        for j in range(0, len(data)):
            # clear all values from sum_values array so it can be populated again
            sum_values.clear()

            # step through given data, storing values that are within filter range
            for distance in range(j-filter_strength, j+filter_strength):
                # if the distance is out of range, set it to extremes of data
                if distance < 0: distance = 0
                if distance > len(data)-1: distance = len(data)-1 

                # add data to list 
                sum_values.append(data[distance])

            # taking the mean of the data
            filtered_values.append(sum(sum_values) / (filter_strength*2 + 1)) 
        
        # plotting filtered data
        plt.plot(time_list, filtered_values)
        
        # clearing filtered data so it can be populated again
        filtered_values.clear()

    plt.legend(["X linear acceleration [m/s^2]", "Y linear acceleration [m/s^2]", "Z angular velocity [rad/s]"])
    plt.grid()
    plt.title("Moving average filtered IMU data, window size = " + str(filter_strength))
    plt.xlabel("Time [nanoseconds]")
    plt.ylabel("Acceleration data")
    plt.show()

def odom_plotter(filename, shape = None):
    _headers, values=FileReader(filename).read_file() 

    first_stamp=values[0][-1]
    time_list = []
    x = []
    y = []
    theta = []

    for line in values:
        time_list.append(line[-1] - first_stamp)
        x.append(line[0])
        y.append(line[1])
        theta.append(line[4])

    # Plotting trajectory
    fig, axs = plt.subplots(2)
    axs[0].plot(x, y, label='Trajectory')
    axs[0].scatter(x[0], y[0], color='green', label=f'Start ({round(x[0], 2)}, {round(y[0], 2)})')
    axs[0].scatter(x[-1], y[-1], color='red', label=f'End ({round(x[-1], 2)}, {round(y[-1], 2)})')
    axs[0].set_title(f'x - vs - y ({shape})')
    axs[0].set_xlabel('x (m)')
    axs[0].set_ylabel('y (m)')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].axis('equal')
    
    # Plotting x and y wrt t
    axs[1].set_title(f'x, y, th- vs -t ({shape})')
    axs[1].plot(time_list, x, label=f'x', color='red')
    axs[1].plot(time_list, y, label=f'y', color='orange')
    axs[1].set_xlabel('Time (ns)')
    axs[1].set_ylabel('Position (m)')
    axs[1].grid(True)
    # Plotting theta wrt t on the same plot but on secondary axis
    ax2 = axs[1].twinx()
    ax2.plot(time_list, theta, label=f'th', color='green')
    ax2.set_ylabel('Angle (rad)')
    lines, labels = axs[1].get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.grid(True)
    ax2.legend(lines + lines2, labels + labels2, loc=0)
    
    fig.tight_layout()
            
def laser_plotter(filename):
    headers, values=FileReader(filename).read_file()
    _headers, odom_values=FileReader(f"odom{filename[5:]}").read_file() 
    ranges = []
    angle_increment = []
    angle_timestamps = []

    current_range_count = 0
    # is the timestamp the reading start time or the broadcast time...
    curr_timestamp = values[0][-1]
    for value in values:
        timestamp = value[-1]
        if timestamp != curr_timestamp:
            angle_timestamps += np.linspace(curr_timestamp, timestamp, current_range_count, endpoint=False).tolist()
            current_range_count = 0
            curr_timestamp = timestamp

        ranges.append(value[0])
        angle_increment.append(value[1])
        current_range_count += 1
        # angle_timestamps.append(value[-1])
    
    xy_time = []
    x = []
    y = []
    theta = []
    for line in odom_values:
        x.append(line[0])
        y.append(line[1])
        theta.append(line[4])
        xy_time.append(line[-1])
    
    current_pos_index = 0

    new_point_x = []
    new_point_y = []
    for i in range(len(angle_timestamps)):
        if ranges[i] > 10:
            continue

        robot_frame_x = ranges[i] * math.cos(angle_increment[i])
        robot_frame_y = ranges[i] * math.sin(angle_increment[i])
        
        while current_pos_index + 1 < len(xy_time) and xy_time[current_pos_index + 1] < angle_timestamps[i]:
            current_pos_index += 1
        
        curr_th = theta[current_pos_index]
        curr_x = x[current_pos_index]
        curr_y = y[current_pos_index]
        new_point_x.append(math.cos(curr_th) * robot_frame_x - math.sin(curr_th) * robot_frame_y + curr_x)
        new_point_y.append(math.sin(curr_th) * robot_frame_x + math.cos(curr_th) * robot_frame_y + curr_y)

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.scatter(new_point_x, new_point_y)
    ax.scatter(x, y)
    # ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
    # ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.set_title("Laser Range Data During Spiral Motion")
    ax.grid(True)
    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        shape = str(filename).split(".")[0].split("_")[-1]

        if "imu" in filename:
            imu_plotter(filename)
        elif "odom" in filename:
            odom_plotter(filename, shape)
        if "laser" in filename:
            laser_plotter(filename)
        else:
            plot_errors(filename)
