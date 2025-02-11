import serial

import re

data = [[0],[0]]

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style


style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.axhline(y = 10, color = 'r', linestyle = '-')

s = serial.Serial()

s.baudrate = 115200

s.port = "COM6"

s.open()


dist_pat = re.compile("(?<=cm: )([+-]?([0-9]*[.])?[0-9]+)")
time_pat = re.compile("(?<=time: )([+-]?([0-9]*[.])?[0-9]+)")
setpoint_pat = re.compile("(?<=setpoint: )([+-]?([0-9]*[.])?[0-9]+)")

setpoint = 10

def update_data():
    global setpoint
    val = s.readline().decode("utf-8");
    dist = re.search(dist_pat, val)
    t = re.search(time_pat, val)
    se = re.search(setpoint_pat, val)
    
    if se != None:
        setpoint = float(se.group(0))
    #print(dist)
    #print(t)

    if dist != None and t !=None:
        data[0].append(float(t.group(0))/1000)
        data[1].append(float(dist.group(0)))
        #print(data)

def animate(i):
    update_data()
    graph_data = data
    xs = graph_data[0][-70:]
    ys = graph_data[1][-70:]
    ax1.clear()
    ax1.axhline(y = setpoint, color = 'r', linestyle = '-')
    ax1.set_xlim(min(xs), max(1,max(xs)))
    ax1.plot(xs, ys)

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
