import multiprocessing
import serial
import matplotlib
import re


s = serial.Serial()

s.port = "COM6"
s.baud = 115200

s.open()

dist = re.compile("(?<=time (s): )(?<float>[+-]?([0-9]*[.])?[0-9]+)")
t = re.compile("(?<=cart_pos (cm): )(?<float>[+-]?([0-9]*[.])?[0-9]+)")

expr = [t, dist]

def read_data():
    if s.inWaiting() > 0:
        return s.readline().decode("ascii")
    return None

def find_matches(data, expr):
    m = []
    for pat in expr:
        m.append(re.search(pat, data))
    return m

def update_vals(vals,data, expr):
    m = find_matches(data, expr)
    if all(m):
        for i in range(len(vals)):
            v:re.Match = float(m(i).group(0))
            vals(i).append(v)




