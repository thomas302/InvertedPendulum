import re
import serial

import matplotlib.pyplot as plt

import numpy as np

import time

# Initialize data
x_data = []
y_data = []

# Create figure and axes
fig, ax = plt.subplots()
line, = ax.plot(x_data, y_data)

# Set initial x and y limits
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)


s = serial.Serial()

s.baudrate = 115200

s.port = "COM6"

s.open()


dist_pat = re.compile("(?<=cm: )([+-]?([0-9]*[.])?[0-9]+)")
time_pat = re.compile("(?<=time: )([+-]?([0-9]*[.])?[0-9]+)")
setpoint_pat = re.compile("(?<=setpoint: )([+-]?([0-9]*[.])?[0-9]+)")

#setpoint = 10

def update_data():
    #global setpoint
    val = s.readline().decode("utf-8");

    dist = re.search(dist_pat, val)
    t = re.search(time_pat, val)
    se = re.search(setpoint_pat, val)

    if dist != None and t !=None:
        return (float(t.group(0))/1000, float(dist.group(0)))
    else:
        return None, None

# Update function
def update(new_x, new_y):
    x_data.append(new_x) 
    y_data.append(new_y)

    # Update x-axis limits to include 0 and the new x value
    min_x = min(x_data)
    max_x = max(1,max(x_data))

    min_y = min(y_data)
    max_y = max(10, max(y_data))

    ax.set_xlim(min_x, max_x) 
    ax.set_ylim(min_y, max_y+1)
    # Update data and redraw
    
    line.set_xdata(x_data)
    line.set_ydata(y_data)
    
    fig.canvas.draw()
    fig.canvas.flush_events()


fig.show()
while True:
    x, y = update_data()
    if x != None:
        update(x, y)
    time.sleep(0.1)
