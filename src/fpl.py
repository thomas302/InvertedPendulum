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


start = 0

dist_pat = re.compile("(?<=cm: )([+-]?([0-9]*[.])?[0-9]+)")
time_pat = re.compile("(?<=time: )([+-]?([0-9]*[.])?[0-9]+)")
setpoint_pat = re.compile("(?<=setpoint: )([+-]?([0-9]*[.])?[0-9]+)")

def update_data():
    global s, start
    #global setpoint
    if s.inWaiting():
        val = s.readline().decode("ascii")

        dist = re.search(dist_pat, val)
        t = re.search(time_pat, val)
        se = re.search(setpoint_pat, val)
        if dist != None and t !=None:
            _s = float(se.group(0)) if se != None else None
            return float(t.group(0)), float(dist.group(0)), _s
        else:
            return None, None, None

fig = fpl.Figure(size=(700,500))

subplot = fig[0,0]

subplot.set_title("Real Time Plot Of Dist")

i = 0
def update(sub=subplot):
    new_x, new_y, _ = update_data()
    if new_x != None and new_y != None:
        x_data.append(new_x)
        y_data.append(new_y)
        data = np.dstack([x_data, y_data])[0]
        sub.add_line(data, colors="b")
        sub.auto_scale(maintain_aspect=False)
         


fig[0, 0].add_animations(update)

c = fig.show()

fpl.loop.run()
