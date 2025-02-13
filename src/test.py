import re
import serial

import matplotlib.pyplot as plt

import numpy as np

import time

from math import sin, sinh, cosh
from math import e

# Initialize data
x_data = []
y_data = []

s = serial.Serial()

s.baudrate = 115200

s.port = "COM6"

s.open()


dist_pat = r"(?<=cm: )([+-]?(\d*[.])?\d+)"
time_pat = r"(?<=time: )([+-]?(\d*[.])?\d+)"
setpoint_pat = r"(?<=setpoint: )([+-]?(\d*[.])?\d+)"

expr = re.compile(dist_pat +r"|"+ time_pat )
def update_data():
    global s
    if s.inWaiting():
        val = s.readline().decode("ascii")
        search = re.findall(expr, val)
        dist = None
        t = None
        se = None
        if len(search) > 0:
            print(search)
            dist = search[0][0]
            t = search[1][2]

            se = None
        if dist != None and t !=None:
            _s = float(se) if se != None else None
            return float(t), float(dist), _s
        return None, None, None
    else:
        return None, None, None


# Create figure and axes
fig, ax = plt.subplots()

line, = ax.plot(x_data, y_data)

hline = ax.axhline(y=1.0, color="r", linestyle="--")
fig.canvas.draw()

# Set x and y limits
ax.set_xlim(0, 20) 
ax.set_ylim(-.1, 1.5)#-40, 40

# Cache background
bg = fig.canvas.copy_from_bbox(fig.bbox)
# Update function
def update(new_x, new_y, new_setpoint):
    x_data.append(new_x) 
    y_data.append(new_y)
 
    line.set_xdata(x_data[-1000:])
    line.set_ydata(y_data[-1000:])

    if max(x_data[-1000:]) > max(plt.xlim()):
        ax.set_xlim(0, max(plt.xlim())+10)

    if new_setpoint != None:
        hline.set_ydata([new_setpoint, new_setpoint]) 
    # Update data and redraw
    fig.canvas.draw()
    fig.canvas.flush_events()


plt.show(block=False)

while True:
    x, y, se = update_data()
    if x != None:
        update(x, y, se)
