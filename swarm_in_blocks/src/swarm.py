#!/usr/bin/python3

import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger
import math
import matplotlib.pyplot as plt
import numpy as np

#Menu 
def menu():
   print("Press")
   print("1 - takeoff all")
   print("2 - line formation")
   print("3 - triangle formation")
   print("4 - square formation")
   print("0 - initial position")
   print("L - land all")
   print("E - exit")

class SingleClover: 
#Create and call all servicers, subscribers and clover topics

   def __init__(self, name, id):
      self.name = name
      self.id = id

      self.current_state = State()

      # Configure clover services and topics
      self.configure()

   def stateCb(self, msg_cb):
      self.current_state = msg_cb

   def configure(self):
      self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)   
      self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
      self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
      self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
      self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
      self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
      self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
      self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
      self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger) 


class Swarm:
   def __init__(self, number_clover):
      rospy.init_node('swarm')
      self.number_clover = number_clover
      self.swarm = []
      self.init_x = []
      self.init_y = []
      self.id_clover = []
      for index in range(number_clover):
         clover_object = SingleClover(f"clover{index}", index)
         self.swarm.append(clover_object)
         self.init_x = self.init_x + [0]
         self.init_y = self.init_y + [index]
         self.id_clover.append(index)
      
   
   def takeoff_all(self):
      coord = np.empty((0,4))
      print("All drones taking off")
      for clover in self.swarm:
         point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
         clover.navigate(x=0, y=0, z=1, auto_arm=True)
         #coord.append([self.init_x[clover.id],self.init_y[clover.id],1,1])
         coord = np.concatenate((coord,[point]))
      plt.plot_preview(coord)
      return coord

   def initial_position(self):
      coord = np.empty((0,4))
      print("All drones returning")
      for clover in self.swarm:
         point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
         clover.navigate(x=0, y=0, z=1)
         coord = np.concatenate((coord,[point]))
      plt.plot_preview(coord)
      return coord

   def land_all(self):
      coord = np.empty((0,4))
      for clover in self.swarm:
         clover.land()
         point = [self.init_x[clover.id],self.init_y[clover.id],0,1]
         coord = np.concatenate((coord,[point]))
      plt.plot_preview(coord)
      return coord
   
   def line(self, z0=1, L=1):
      coord = np.empty((0,4))
      N = self.number_clover
      f = L/(N-1)
      print("Beginning line formation")
      for clover in self.swarm:
         x0 = 0 - self.init_x[clover.id]
         y0 = 0 - self.init_y[clover.id]
         point = [round(f*(N-1-clover.id),2), 0, z0, 1]
         clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
         #clover.navigate(x=(x0+f*(N-1-clover.id)), y=y0, z=z0)
         #coord.append([round(x0+f*(N-1-clover.id), 2), 0, z0, 1])
         coord = np.concatenate((coord,[point]))
         rospy.sleep(2)
      plt.plot_preview(coord)
      rospy.sleep(5)
      print("Line done\n")
      return coord

   def square_side(self, q, n, yi, L):
      j = 0
      if (n == 1):
         f = L/2
         j = -1
      else:
         f = L/(n-1)
      for clover in self.swarm[q:n+q]:
         x0 = 0 - self.init_x[clover.id]
         y0 = 0 - self.init_y[clover.id]
         clover.navigate(x=(x0+f*(n-1-j)), y=y0+yi, z=z0)
         q = q+1
         j = j+1
         rospy.sleep(2)
         if (q==N):
               break
      return(q)

   def square(self, type="full", z0=1, L=2):
      coord = []
      print("Beginning square formation")
      yi = 0
      n = int(1 + N/4)

      if (type=="empty"):
         if (N%4 == 0):
            q = self.square_side(q=0, n=n, yi=0, L=L)
            while (q<N-n):
                  yi = yi + L/(n-1)
                  q = self.square_side(q=q, n=2, yi=yi, L=L)
            q = self.square_side(q=q, n=n, yi=L, L=L)
         else:
            q = self.square_side(q=0, n=n+1, yi=0, L=L)
            if (N%4 > 1):
                  m = n+1
            else:
                  m = n
            while (q<N-n):
                     yi = yi + L/(m-1)
                     q = self.square_side(q=q, n=2, yi=yi, L=L)
            if (N%4 < 3):
                  q = self.square_side(q=q, n=n, yi=L, L=L)
            else:
                  q = self.square_side(q=q, n=n+1, yi=L, L=L)                

      elif (type=="full"):
         q = self.square_side(q=0, n=n, yi=0, L=L)
         while (q<N):
            if (math.sqrt(N) == int(math.sqrt(N))):
                  yi = yi + L/(n-1)
                  q = self.square_side(q=q, n=n, yi=yi, L=L)
            else:
                  yi = yi + L/n
                  q = self.square_side(q=q, n=(N%4), yi=yi, L=L)
                  if (N-q == n):
                     q = self.square_side(q=q, n=n, yi=L, L=L)

      rospy.sleep(5)
      print("Square done\n")

   def circle(self, xc=4, yc=4, z0=1, r=2):
      coord = np.empty((0,4))
      print("Beginning circle formation")
      angle = 2*math.pi/N
      for clover in self.swarm:
         x0 = 0 - self.init_x[clover.id]
         y0 = 0 - self.init_y[clover.id]
         xi = r*math.cos(clover.id*angle)
         yi = r*math.sin(clover.id*angle)
         point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
         clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
         # clover.navigate(x=x0+xc+xi, y=y0+yc+yi, z=z0)
         # coord.append([(x0+xc+xi), (y0+yc+yi), z0,1])
         coord = np.concatenate((coord,[point]))
         rospy.sleep(5)
      plt.plot_preview(coord)
      rospy.sleep(5)
      print("Circle done\n")
      return coord

      
   def triangle(self, x0 = 0, y0 = 0, z0 = 1):
      coord = []
      N = self.number_clover
      L=1
   
      for index in range(N):
         if(index%3==0):
            L+=1
      if(N%3==0 and N>3):
         L-=1
   
      f = (math.sqrt(3)*L)/2
      c1=0
      c2=0
      reta = math.sqrt(3)
      boss_clover = int(np.median(self.id_clover))
      print (boss_clover)
      print("Beginning triangle formation")
      for clover in self.swarm:
         if(clover.id%2 != 0):
            if(clover.id == int(np.median(self.id_clover))):
               print("hello if 1")
               clover.navigate(x=x0+f, y = y0, z = z0)
               
            else:
               c1 = 1/2
               print("hello if 1.2")
               clover.navigate(x=x0 +reta*c1, y = y0, z = z0)
            
            if(clover.id==N-1):
                     clover.navigate(x=x0,  y = y0 - self.init_y[N-boss_clover-1], z = z0+2)
                     rospy.sleep(5)
                     clover.navigate(x=x0,  y = y0 - self.init_y[N-boss_clover-1], z = z0)
               
   
         if(clover.id%2 == 0):
            if(clover.id == 0):
               print("hello if 2") 
               clover.navigate(x=x0, y=y0, z=z0)
               c2 += 1/2
               
            elif(L-reta*c2<f):
               print("hello if 2.1")
               if(clover.id == int(np.median(self.id_clover))):
                  print("hello if 2.2")
                  clover.navigate(x=x0+f, y = y0, z = z0)
                  
   
               else:
                  print("hello if 2.3")
                  clover.navigate(x=x0 + L - reta*c2, y = y0 + self.init_y[N-clover.id-1], z = z0)
                  c2 += 1/2
   
                  if(N%2 == 0):
                     N-=1
                  
                  if(clover.id==N-1):
                     clover.navigate(x=x0,  y = y0 + self.init_y[N-clover.id-1], z = z0)
                     N+=1    

   # def formations(self,type):
   #    if (type == "line"):
   #       self.line(z0=1, L=1)
      
   #    elif (type == "square"):
   #       self.square(type="full", z0=1, L=2)

   #    elif (type == "triangle"):
   #       self.triangle(x0=0, y0=0, z0=1, L=1)
      
   #    return 

   def launchSwarm(self):
      pass



if __name__ == "__main__":

   swarm = Swarm(6)
   N = swarm.number_clover

   while not rospy.is_shutdown():
      menu()
      key= input("\n")
      if (key == str('1')):
         coord = swarm.takeoff_all()
         print("Drones coordinates: \n{}\n".format(coord))
         rospy.sleep(2)
      elif (key == str('2')):
         if (N < 2):
               print("You need at least 2 clovers!\n")
         else:
               z0 = int(input("Insert the desired height: "))
               L = int(input("Insert the desired length: "))
               coord = swarm.line(z0=z0, L=L)
               print("Drones coordinates: \n{}\n".format(coord))

               rospy.sleep(5)
      elif (key == str('3')):
         if (N < 3):
               print("You need at least 3 clovers!\n")
         else:
               x0 = int(input("Insert initial x coordinate: "))
               y0 = int(input("Insert initial y coordinate: "))
               z0 = int(input("Insert the desired height: "))
               L = int(input("Insert the desired side length: "))
               swarm.triangle(x0=x0, y0=y0, z0=z0)
               rospy.sleep(5)

      elif (key == str('4')):
         if (N < 4):
               print("You need at least 4 clovers!\n")
         else:
               type = input("Insert full or empty: ")
               z0 = int(input("Insert the desired height: "))
               L = int(input("Insert the desired side length: "))
               swarm.square(type=type, z0=z0, L=L)
               rospy.sleep(5)

      elif (key == str('o') or key == str('O')):
         coord = swarm.circle(z0=1, r=2)
         print("Drones coordinates: \n{}\n".format(coord))
         rospy.sleep(2)

      elif (key == str('0')):
         coord = swarm.initial_position()
         print("Drones coordinates: \n{}\n".format(coord))
         rospy.sleep(2)

      elif (key == str('l') or key == str('L')):
         coord = swarm.land_all()
         print("Drones coordinates: \n{}\n".format(coord))
         rospy.sleep(5)

      elif (key == str('e') or key == str('E')):
         break


