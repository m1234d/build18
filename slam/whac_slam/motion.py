import serial
import time
data = serial.Serial("/dev/ttyACM0", 9600)
time.sleep(2)

data.write('right;')
response = data.readline()
print(response)

time.sleep(.5)
data.write('stop;')
response = data.readline()
print(response)
