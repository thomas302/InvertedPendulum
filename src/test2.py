import serial
import re

s = serial.Serial()

s.baudrate = 115200

s.port = "COM6"

s.open()


setpoint_pat = re.compile("(?<=setpoint: )([+-]?([0-9]*[.])?[0-9]+)")
while True:
    val = s.readline().decode("utf-8")
    print(val)
    match = re.search(setpoint_pat, val)

    if match != None:
        print("Setpoint: ", float(match.group(0)))

    
