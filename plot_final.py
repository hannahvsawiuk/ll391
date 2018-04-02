import matplotlib.pyplot as plt
import time
import serial
import csv

plt.ion()
ser = serial.Serial(port='COM3', baudrate=57600, timeout=1 )
line = ser.readline() # throw away any part lines
while(ser.inWaiting() < 100): # make sure something is coming
  now = 0.0
t=[] # initialize the data lists
d1=[]
d2=[]
d3=[]
d4=[]
d5=[]
d6=[]
while (ser.isOpen()):
  # line = ser.read(4) # ead a line of text
  line = ser.readline()
  # print(line)
  line = line.decode('ascii')
  # print(line)
  # line = line.strip("\n")
  # print(line)
  # bpm,sp02 = line.split(",")
  line = line.strip()
  mylist = line.split(",")
  # print(bpm,sp02)
  # print(line)
  # print(mylist)
  now = float(mylist[0])/1000 # time now in seconds
  t.append(float(mylist[0])/1000) # from first element as milliseconds
  d1.append(float(mylist[1])) # six data elements added to lists
  # d2.append(float(mylist[2]))
  # d3.append(float(mylist[3]))
  # d4.append(float(mylist[4]))
  # d5.append(float(mylist[5]))
  # d6.append(float(mylist[6]))
  if(ser.inWaiting() < 100): # redraw only if you are caught up
    plt.clf() # clear the figure
    plt.plot(t,d1) # plot a line for each set of data
    # plt.plot(t,d2)
    # plt.plot(t,d3)
    # plt.plot(t,d4)
    # plt.plot(t,d5)
    # plt.plot(t,d6)
    plt.axis([now-60,now,min(d1)-50,max(d1)+50])
    plt.xlabel("Time Since Boot [s]")
    plt.draw()
