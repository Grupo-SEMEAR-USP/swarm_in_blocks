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
import formation

#Menu 
def menu():
   print("Press")
   print("1 - takeoff all")
   print("2 - line formation")
   print("3 - triangle formation")
   print("4 - square formation")
   print("O - circle formation")
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
   def __init__(self, num_of_clovers):
      rospy.init_node('swarm')
      self.num_of_clovers = num_of_clovers
      self.swarm = []
      self.init_x = []
      self.init_y = []
      self.id_clover = []

      # Create clover python objects
      self.__createCloversObjects()

      # Current formation name and coord in homogeneous line vector type 
      # Ex of self.curr_form_coords: [[x0,y0,z0,1],[x1,y1,z1,1], [x2,y2,z2,1], ...]
      self.curr_formation_name = ''
      self.curr_formation_coords = []

      # Lists of formations. Can be used to plan a formation changing for formation that is hard to generate in real time.
      self.formation_list = []

      # Leader id of the swarm
      self.leader_id = None

   # Create clover objects and append to clover object list
   def __createCloversObjects(self):
      for index in range(self.num_of_clovers):
         clover_object = SingleClover(f"clover{index}", index)
         self.swarm.append(clover_object)
         self.init_x = self.init_x + [0]
         self.init_y = self.init_y + [index]
         self.id_clover.append(index)
   
   def launchGazeboAndClovers(self):
      pass

   def takeoff_all(self):
      coord = formation.takeoff_all(self)
      return coord

   def initial_position(self):
      coord = formation.initial_position(self)
      return coord

   def land_all(self):
      coord = formation.land(self)
      return coord

   def line(self, z0, L):
      coord = formation.line(self, z0, L)
      return coord

   def square(self, type, z0, L):
      coord = formation.square(self, type, z0, L)
      return coord

   def setLeader(self, id):
      
      assert(type(id)==int, "Input 'id' must be an integer.")
      self.leader_id = id
   
   def followLeader():
      pass

   #Função temporária aqui, só apagar quando testes com a triangle estiverem oks
   def square_side(self, q, n, yi, L, coord):
      j = 0
      if (n == 1):
         f = L/2
         j = -1
      else:
         f = L/(n-1)
      for clover in self.swarm[q:n+q]:
         x0 = 0 - self.init_x[clover.id]
         y0 = 0 - self.init_y[clover.id]
         point = [round(f*(n-1-j),2), yi, z0, 1]
         clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
         #clover.navigate(x=(x0+f*(n-1-j)), y=y0+yi, z=z0)
         coord = np.concatenate((coord,[point]))
         q = q+1
         j = j+1
         rospy.sleep(2)
         if (q==N):
               break
      return(q, coord)


   def circle(self, xc, yc, z0, r):
      coord = formation.circle(self, xc, yc, z0, r)
      return coord
      
   def triangle(self, x0 = 0, y0 = 0, z0 = 1):
      coord = np.empty((0,4))
      N = self.num_of_clovers
      L=2

      for index in range(N):
         if(index%3==0 and N>3):
            L+=1

      #Verificação para que o L seja incrementado sempre após a adição de 3 drones
      if(index%3==0 and N>3):
        f = (math.sqrt(3)(L-1))/2

      else:
         f = (math.sqrt(3)*L)/2
      
      c1=0
      #c2=0
      reta = math.sqrt(3)
      boss_clover = int(np.median(self.id_clover))
      print (boss_clover)
      print("Beginning triangle formation")
      for clover in self.swarm:
         
         if(clover.id%2 != 0): 
            #c1 += 1/2
            if(clover.id == int(np.median(self.id_clover))):
               print("hello if 1")
               point = [round(f,2), 0, z0, 1]
               clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
               #clover.navigate(x=x0+f, y = y0, z = z0)
               
            else:
               print("hello if 1.2")
               point = [round(reta*c1,2), 0, z0, 1]
               clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
               #clover.navigate(x=x0 +reta*c1, y = y0, z = z0)
            
            if(clover.id==N-1):
               point = [0, self.init_y[N-boss_clover-1], z0, 1]
               clover.navigate(x=x0+point[0], y=y0-point[1], z=point[2]+2)
               #clover.navigate(x=x0,  y = y0 - self.init_y[N-boss_clover-1], z = z0+2)
               rospy.sleep(5)
               #clover.navigate(x=x0,  y = y0 - self.init_y[N-boss_clover-1], z = z0)
               clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
               
   
         if(clover.id%2 == 0):
            c1 += 1/2
            if(clover.id == 0):
               print("hello if 2") 
               point = [0, 0, z0, 1]
               clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
               #clover.navigate(x=x0, y=y0, z=z0)
               #c2 += 1/2
               
            else:
               print("hello if 2.1")
               #c2 += 1/2
               if(clover.id == int(np.median(self.id_clover))):
                  print("hello if 2.2")
                  point = [f, 0, z0, 1]
                  clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
                  #clover.navigate(x=x0+f, y = y0, z = z0)
                 
   
               else:
                  print("hello if 2.3")
                  #c2 += 1/2
                  point = [round(L-reta*c1,2), self.init_y[N-clover.id-1], z0, 1]
                  clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
                  #clover.navigate(x=x0 + L - reta*c2, y = y0 + self.init_y[N-clover.id-1], z = z0)
   
                  if(N%2 == 0):
                     N-=1
                  
                  if(clover.id==N-1):
                     point = [0, self.init_y[N-clover.id-1], z0, 1]
                     clover.navigate(x=x0+point[0], y=y0+point[1], z=point[2])
                     #clover.navigate(x=x0,  y = y0 + self.init_y[N-clover.id-1], z = z0)
                     N+=1
         coord = np.concatenate((coord,[point]))
      plot_preview(coord)
      rospy.sleep(2)
      print("Circle done\n")
      return coord

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

def plot_preview(coord):
   plt.plot(coord[:,0],coord[:,1],'ro')
   plt.axis([-1,11,-1,11])
   plt.grid(True)
   plt.show()

if __name__ == "__main__":

   swarm = Swarm(6)
   N = swarm.num_of_clovers

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
               # x0 = int(input("Insert initial x coordinate: "))
               # y0 = int(input("Insert initial y coordinate: "))
               # z0 = int(input("Insert the desired height: "))
               # L = int(input("Insert the desired side length: "))
               swarm.triangle()
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
         r = int(input("Insert the desired ratio: "))
         xc = int(input("Insert the center x coordinate: "))
         yc = int(input("Insert the center y coordinate: "))
         z0 = int(input("Insert the desired height: "))
         coord = swarm.circle(xc=xc, yc=yc, z0=z0, r=r)
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



