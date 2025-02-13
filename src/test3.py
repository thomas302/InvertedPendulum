import re
import serial
import fastplotlib as fpl
import numpy as np
import time

# Initialize data
x_data = []
y_data = []

s = serial.Serial()

s.baudrate = 115200
s.port = "COM6"
s.open()

dist_pat = re.compile("(?<=cm: )([+-]?([0-9]*[.])?[0-9]+)")
time_pat = re.compile("(?<=time: )([+-]?([0-9]*[.])?[0-9]+)")
setpoint_pat = re.compile("(?<=setpoint: )([+-]?([0-9]*[.])?[0-9]+)")

def update_data():
    global s
    val = s.readline().decode("ascii")

    dist = re.search(dist_pat, val)
    t = re.search(time_pat, val)
    se = re.search(setpoint_pat, val)
    if dist != None and t != None:
        _s = float(se.group(0)) if se != None else None
        return float(t.group(0)), float(dist.group(0)), _s
    else:
        return None, None, None

fig = fpl.Figure(size=(700, 500))

# Initially, set up empty arrays for the plot
line = fig[0, 0].add_line(np.empty((0, 2)), name="pos", colors="r")  # Empty 2-column array for x and y data

subplot = fig[0, 0]

subplot.set_title("Real Time Plot Of Dist")
subplot.ylim = (0, 1.5)

def update(sub=subplot):
    new_x, new_y, _ = update_data()
    if new_x is not None and new_y is not None:
        x_data.append(new_x)
        y_data.append(new_y)

        # Create the correct 2D array using np.array with the correct shape (n_points, 2)
        data = np.array(list(zip(x_data, y_data)))  # Zip x_data and y_data into pairs
        
        print(f"Data being passed to the plot: {data.shape}")  # Check shape of data
        
        # Update the plot data with the correct shape
        sub["pos"].data = data

fig[0, 0].add_animations(update)

fig.show(maintain_aspect=False)

fpl.loop.run()

