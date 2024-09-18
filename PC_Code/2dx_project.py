

#   Currently set COM5 as serial port at 115.2kbps 8N1 with a 10 timeout period

import serial
import numpy as np
import open3d as o3d
import math

f = open("tof_radar.xyz", "w")

s = serial.Serial('COM5', 115200, timeout = 10) #Change your port here as desired

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

x_displacement = int(input("Please enter the x displacement value in mm: ")) #enter the distance in between each scan
num_displacements = int(input("Please enter the number of displacements taking place: ")) #enter the number of scans with x mm displacement

# wait for user's signal to start the program
input("Press Enter to start communication...")
s.write(b's')
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
for displacement in range(num_displacements):   # loop repeats for the number of displacements entered at the beginning of the program
    print("Collecting displacement ", displacement + 1 , " data:")
    valid_measurements = 0 
    #measurements are taken every 5.625 degrees, requiring 64 scans per 360 degrees
    while valid_measurements < 64: # a valid measurement contains actual distance data.
        x = s.readline() # reads from the UART line
        decoded_x = x.decode('utf-8').strip() # formats the measurement so it can be read as a number
        print(decoded_x) # prints each measurement to the terminal
        if decoded_x.isdigit():  # Check if the string represents an integer, if so it is a valid measurement
            distance = int(decoded_x)
            xvalue = displacement * x_displacement # x is incremented by the x_displacement value entered earlier in the program
            yvalue = distance * math.sin(math.radians(5.625*valid_measurements)) # y and z are calculated using trigonometry. Angle increments by 5.625 degrees each measurement 
            zvalue = distance * math.cos(math.radians(5.625*valid_measurements))  
            f.write(f"{xvalue} {yvalue} {zvalue}\n") #  write each coordinate to the .xyz file
            valid_measurements += 1  # Increment only if the data was valid
        else:
            print("Waiting for valid data...")
#close the file
f.close()
#close the port
print("Closing: " + s.name)
s.close()

pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz") 

# Initialize a list to store the lines
lines = []

# Loop to create lines within each set and to close the loop of each set
for set_number in range(num_displacements):
    start_index = set_number * 64
    for i in range(64):
        if i < 63:  # Connect each point to the next within the set
            lines.append([start_index + i, start_index + i + 1])
        else:  # Connect the last point to the first to close the loop
            lines.append([start_index + i, start_index])

# Loop to connect corresponding points between each set to form a continuous path
for set_number in range(num_displacements - 1):
    for i in range(64):
        current_index = set_number * 64 + i
        next_index = (set_number + 1) * 64 + i
        lines.append([current_index, next_index])

# Create a LineSet object using the points from the point cloud and the lines list
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
    lines=o3d.utility.Vector2iVector(lines)
)

# Visualize the point cloud and the line set together
o3d.visualization.draw_geometries([pcd, line_set])
