import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time
import serial
import csv

plt.ion()
ser = serial.Serial(port='COM3', baudrate=57600, timeout=1 )
line = ser.readline() # throw away any part lines
while(ser.inWaiting() < 100): # make sure something is coming
  now = 0.0

t         = [] # initialize the data lists
desired   = []
actual    = []
error     = []

while (ser.isOpen()):
  line    = ser.readline()
  line    = line.decode('ascii')
  line    = line.strip()
  mylist  = line.split(",")
  now     = float(mylist[2])/1000 # time now in seconds

  t.append(now)
  desired.append(float(mylist[0])/100) 
  actual.append(float(mylist[1])/100) 
  error.append(abs((float(mylist[0]) - float(mylist[1]))/1000))

  if(ser.inWaiting() < 100): # redraw only if you are caught up
    # plt.clf() # clear the figure
    
    desLine, = plt.plot(t, desired, color='purple') # plot a line for each set of data
    actLine, = plt.plot(t, actual, color='blue')
    errLine, = plt.plot(t, error, color='green')

    ymin = (min(desired) + min(actual))/2 
    ymax = (max(desired) + max(actual))/2 
    plt.axis([now-10,now+50,ymin-25,ymax+25])
    plt.xlabel("Time [s]")

    desPatch = mpatches.Patch(color='purple', label='Desired')
    actPatch = mpatches.Patch(color='blue', label='Actual')
    errPatch = mpatches.Patch(color='green', label='Error')
    plt.legend(handles=[desPatch,actPatch,errPatch])

    plt.draw()
    plt.pause(0.05)




