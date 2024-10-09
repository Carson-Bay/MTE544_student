# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader

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
            

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        if "imu" in filename:
            imu_plotter(filename)
        else:
            plot_errors(filename)
