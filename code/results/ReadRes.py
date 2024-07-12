import numpy as np
from ax12 import Ax12
from itertools import product
import time
import socket
#Load data from file
a = np.load('results.npy')
#Initialize variables and state table
Dest = [0,0,0]
st=[0,0]
M1O=2000
M2O=1850
Stateval = np.arange(0,1250,50)
s_table = list(product(Stateval, repeat=2))

#Set PID gains on servos
M=Ax12()
M.setPgain(1,128)
time.sleep(0.2)
M.setPgain(2,128)
time.sleep(0.2)
M.setIgain(1,66)
time.sleep(0.2)
M.setIgain(2,66)
time.sleep(0.2)
M.setDgain(1,200)
time.sleep(0.2)
M.setDgain(2,200)
time.sleep(0.2)
#Coin detection
def acqimg():
  Rport = 12345
  data = ''
  for a in range(10):
    try:
      d = socket.socket()
      d.bind(('0.0.0.0', Rport))
      d.listen(5)
      x, addr = d.accept()
      while True:
        data = x.recv(1024)
        if data !='':
          #format of data: int list [ammount, x, y, diameter]
          data = data.split(',')
          data = list(map(int, data))
          break
      d.close()
    except socket.error, e:
        time.sleep(1)
        continue
    else:
        return data
        break
#Robot call
def callrob(coinL, process):
  coindata = '['+str(coinL[0])+','+str(coinL[1])+','+str(coinL[2])+']'
  ABBhost = '130.230.141.123'
  ABBport = 1025
  c = socket.socket()
  c.connect((ABBhost, ABBport))
  c.sendall(bytes(process))
  ans = c.recv(1024)
  c.sendall(bytes(coindata))
  ans = c.recv(1024)
  if process == 1:
    print ('Going to: '+repr(ans))
  if process == 2:
    print ('Moving Robot away')
  while True:
    ans = c.recv(1024)
    if ans == str('Done'):
      c.close()
      print ('Robot in place')
      break
    else:
      c.sendall('Ok')
#Move the gripper    
def MoveG(M1,M2):
    try:
        M = Ax12()
        mov1 = 0
        mov2 = 0
        M.move(1, M1)
        M.move(2, M2)
        mov1 = M.readMovingStatus(1)
        time.sleep(0.05)
        mov2 = M.readMovingStatus(2)
        time.sleep(0.05)
        while mov1==1 or mov2==1:
            mov1 = M.readMovingStatus(1)
            time.sleep(0.05)
            mov2 = M.readMovingStatus(2)
            time.sleep(0.05)
    except:
        mov1=1
        mov2=1

#To read the sequence on file 
def CloseGripper(st):
    moves=0
    NState=s_table.index((st[0],st[1]))
    while st[0]<1200 or st[1]<1200:
        if a[NState] == 0:
            st = [st[0],st[1]+50]
        if a[NState] == 1:
            st = [st[0]+50,st[1]]
        if a[NState] == 2:
            st = [st[0]+50,st[1]+50]
        if a[NState] not in range(0,3):
            st=[0,0]
        if st[0] > 1200:
            st[0]=1200
        if st[1] > 1200:
            st[1]=1200
        MoveG(M1O-st[0],M2O-st[1])
        moves+=1
        print(st)
        NState=s_table.index((st[0],st[1]))
    print('Number of movements: '+str(moves))
        

#Show table
print(a)
#Open gripper
MoveG(M1O,M2O)
#Detect coin in environment
coin = acqimg()
coinL= coin[1:3]
coinL.append(0)
#If there is any coin
if coin[0] > 0:
    #Go to coin position
    callrob(coinL,1)
    #Close the gripper reading the file
    CloseGripper(st)
    #Go away
    callrob(Dest,2)
    #Drop the coin
    MoveG(M1O,M2O)
else:
    print('No coin detected')
    