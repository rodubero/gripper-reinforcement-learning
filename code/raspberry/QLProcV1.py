#****************************************************
#Tampere University
#Futre Automation Systems and Technologies Laboratory
#FAST-LAB
#
#Q-Learning applied to underactuated gripper control 
#
#This script orchestrate an ABB IRB140 robot, a NI1774C smart camera
#aiming to find a sequence of movements to pick a coin using a
#2 finger underactuated gripper (Openhand T42 - Yale)
#
#The sequence is found by discretizing incremental movements of each finger
#and implement Q-Learning
#
#Results are stored as results.npy (numpy file) and can be read by the
#actuating script
#
#V0:21.08.2019  - RB
#****************************************************
#Import libraries
import random
import socket
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from itertools import product
from ax12 import Ax12
import time

# For controlling the gripper, set P, I, D Gains and positions
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
M1O=2000
M2O=1850
M1C=800
M2C=600
Ran=M1O-M1C

# Hyperparameters
alpha = 0.3
gamma = 0.7
epsilon = 0.3
epomax = 50
epimax = 500

#Environment
q_table = np.zeros([625, 3])
#625 states. 1200 position units range in increments of 50 = 25. 25 positions for each finger: 25*25=625 states
#3 possible actions, move -50 on each single finger or both
Stateval = np.arange(0,Ran+50,50)
s_table = list(product(Stateval, repeat=2))
#Table of states with all possible states of both fingers
histrew=np.array([0])
#Total reward variable
totreward=0

#Define functions to operate robot, camera and gripper
#Script to capture image using NI1774C and a TCP/IP socket server. 
#Cam streams as UDP messages to RPi with ammount of coins, coordinates and 
#diameter of coin. Function returns data as list
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

#Function to call the IRB140 ABB robot using TCP/IP sockets
#coinL are coordinates [X,Y,Z] of the target. process is 1 when the robot 
#aproaches the coin and 2 when moves away of it
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
#Function to move the gripper to positions M1 and M2 
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
#Function to return the coin to the work area
def retCoin():
    Dropc=[50,30,0]
    callrob(Dropc,1)
    MoveG(M1O,M2O)
    callrob(Dest,2)
    
#Define functions for QLearning

#Function to estimulate and action, requires Current state (Cstate), 
#action (action) and a Table of all possible states (Stable)
def step(Cstate, action, Stable):
  #Reads the current state
  st=[Stable[Cstate][0],Stable[Cstate][1]]
  #Applies the action requested
  if action == 0:
    st = [st[0],st[1]+50]
  if action == 1:
    st = [st[0]+50,st[1]]
  if action == 2:
    st = [st[0]+50,st[1]+50]
  if action not in range(0,3):
    st=[0,0]
  #Every action gives reward of -1
  reward=-1
  #If the closing point is reached, process is finnished
  if st[0]>=Ran and st[1]>=Ran:
    done = True
    NState=Stable.index((Ran,Ran))
    return NState, reward, st, done
  else:
    done = False
  if not st[0] in range(0,1201) or not st[1] in range(0,1201):
        reward =-5
  #Evaluates if it reached the limit positions of the servomotors
  if st[0]>Ran: 
    st[0]=Ran
  if st[1]>Ran: 
    st[1]=Ran
  if st[0]<0:
    st[0]=0
  if st[1]<0:
    st[1]=0
  NState=Stable.index((st[0],st[1]))
  return NState, reward, st, done

Epis=0
#Training loop
while Epis < epimax:
  #Detect position of the coin and store it in coinL
  Dest = [0,0,0]
  coin = acqimg()
  coinL= coin[1:3]
  coinL.append(0)
  totreward=0
  #If any coin is found send the robot to it and start episode
  if coin[0] > 0:
    #Initialize episode. Initialize variables, open Gripper
    epok = False
    MoveG(M1O,M2O)
    CState=0
    callrob(coinL,1)
    epochs, penalties, reward, = 0, 0, 0
    done = False
    epok = False
    #While is not on closed position or has not reached max amount of epochs
    while not done and not epok:
      #randomly select action
      if random.uniform(0, 1) < epsilon:
        action = np.random.randint(0,3) # out of 3 possible actions
      else:
        action = np.argmax(q_table[CState]) # Exploit learned values
      #Execute the action selected    
      next_state, reward, counts, done = step(CState, action, s_table)
      MoveG(M1O-counts[0],M2O-counts[1])
      #If the gripper is completely closed, check if it picked the coin
      if done or epochs >= epomax:
        epok=True
        #Take the robot away
        callrob(Dest,2)
        #Look for coin
        coinp = acqimg()
        #If it picked the coin
        if coinp[0] == 0:
          #Gives best reward 20
          reward = 30
          #Drop the coin back to keep training
          retCoin()
        else:
            reward = -50
      #Get current probability value  
      old_value = q_table[CState, action]
      #Find value of maximum probability for next state
      next_max = np.max(q_table[next_state])
      #Calculate new probability for current state, selected action    
      new_value = ((1 - alpha) * old_value) + (alpha * (reward + (gamma * next_max)))
      q_table[CState, action] = new_value
      #Go to next state, advance 1 epoch
      CState = next_state
      epochs += 1
      totreward=totreward+reward
      print('Epoch: '+str(epochs), ' Total reward: '+str(totreward))
      #If max epochs reached, remove the robot and check if it picked the coin
    print("Episode: "+str(Epis))
    histrew = np.append(histrew,totreward)
    Epis+=1
  else:
    try:
        input("No coin detected, please insert coin and press Enter")
    except SyntaxError:
        pass
#Save results
FinalRes=np.argmax(q_table, axis=1)
np.save('results.npy', FinalRes)
np.save('Qres.npy', q_table)
#Present reward vs attempts plot
atmp= np.arange(0,epimax+1,1)
histrew=np.delete(histrew,0)
atmp=np.delete(atmp,0)
fig, ax = plt.subplots()
ax.plot(atmp,histrew)
ax.set(xlabel='Attempts', ylabel='Total reward', title='Total reward evolution')
ax.grid()
fig.savefig('REvsAt.png')
print("Training finished.\n")