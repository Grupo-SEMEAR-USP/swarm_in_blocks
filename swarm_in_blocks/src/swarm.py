#!/usr/bin/python3

import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import formation
import launch

#Menu 
def menu():
   print("Press")
   print("1 - takeoff all")
   print("2 - line formation")
   print("3 - triangle formation")
   print("4 - square formation")
   print("5 - cube formation")
   print("6 - sphere formation")
   print("7 - piramide formation")
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

      self.init_formation = []

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
         
   
   def launchGazeboAndClovers(self):
      launch.spawnGazeboAndVehicles(self.num_of_clovers)

   def applyFormation(self, coord):
      for idx, clover in enumerate(self.swarm):
         clover.navigate(x=coord[idx][0],y=coord[idx][0],z=coord[idx][2])

   #Preview formations
   def plot_preview(self, coord):
      #start_form='False'
      plt.figure(figsize=(8, 8))
      plt.subplots_adjust(bottom = 0.2)
      plt.plot(coord[:,0],coord[:,1],'ro')
      plt.axis([-1,11,-1,11])
      plt.grid(True)
      plt.xticks(np.linspace(0,10,11))
      plt.yticks(np.linspace(0,10,11))
      posit = plt.axes([0.4, 0.1, 0.2, 0.05])
      button = Button(posit,'Confirm')
      #button.on_clicked(start_form='True')
      plt.show(block=False)
      #return start_form

   def plot_preview_3d(self, coord):
      #start_form='False'
      fig = plt.figure(figsize=(8, 8))
      ax = fig.add_subplot(111,projection='3d')
      plt.subplots_adjust(bottom = 0.2)
      ax.plot(coord[:,0],coord[:,1],coord[:,2],'ro')
      #plt.axis([-1,11,-1,11])
      plt.grid(True)
      plt.xticks(np.linspace(0,10,11))
      plt.yticks(np.linspace(0,10,11))
      posit = plt.axes([0.4, 0.1, 0.2, 0.05])
      button = Button(posit,'Confirm')
      #button.on_clicked(start_form='True')
      plt.show(block=False)
      #return start_form

   #def setFormation(self, formation):
      

   #Basic swarm operations
   def takeoff_all(self):
      coord = np.empty((0,4))
      print("All drones taking off")
      for clover in self.swarm:
         point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
         clover.navigate(x=0, y=0, z=1, auto_arm=True)
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      return coord

   def initial_position(self):
      coord = np.empty((0,4))
      print("All drones returning")
      for clover in self.swarm:
         point = [self.init_x[clover.id],self.init_y[clover.id],1,1]
         clover.navigate(x=0, y=0, z=1)
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      return coord

   def land_all(self):
      coord = np.empty((0,4))
      for clover in self.swarm:
         clover.land()
         point = [self.init_x[clover.id],self.init_y[clover.id],0,1]
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      return coord

   #Formations
   def line(self, N, L):
      self.coord = formation.line(N, L)

   def square(self, N, type, L):
      self.coord = formation.square(N, type, L)

   def circle(self, N, xc, yc, r):
      self.coord = formation.circle(self, N, xc, yc, r)
      
<<<<<<< HEAD
   def triangle(self, x0 = 0, y0 = 0, z0 = 1):

      coord = np.empty((0,4))
      N = self.num_of_clovers
      L=2
      reta = np.sqrt(3)
      for index in range(N):
         if((index-1)%3==0 and index>4):
            L+=1

      f = (np.sqrt(3)*L)/2

      c1=0

      boss_clover = int(np.median(self.id_clover))
      if(N==4):
         boss_clover+=1

      print(boss_clover)

      for clover in self.swarm:
        
         if(clover.id<boss_clover):
            
            point = [round(reta*c1,2), 0, z0, 1]
            clover.navigate(x=x0+point[0], y=point[1], z=point[2])
            c1+=1/2
            

         else:
            if(clover.id==int(np.median(self.id_clover)) and N>3):
               c1=0

            if(N==4):
               y0-=1
            point = [round(f-reta*c1,2), 0, z0, 1]
            clover.navigate(x=x0+point[0], y=point[1], z=point[2])
            c1+=1/2
            
            if(clover.id == N-1 and N%2==0):
               if(N>4):
                  clover.navigate(x=x0, y = y0, z=z0+1)
                  y0 = y0 - self.init_y[clover.id]
                  rospy.sleep(5)
                  point = [x0, y0 + self.init_y[boss_clover], z0, 1]
                  clover.navigate(x=point[0], y = point[1], z=point[2])
               else: 
                  clover.navigate(x=x0, y = 0, z=z0+1)
                  rospy.sleep(5)
                  clover.navigate(x=x0, y = y0, z=z0)

         coord = np.concatenate((coord,[point]))
      plot_preview(coord)
      rospy.sleep(2)
      print("Triangle done\n")
      return coord  

   #3D Formations
=======

   def triangle(self):
      coord = formation.triangle(self, self.num_of_clovers)
      return coord
      

>>>>>>> 6b08a0924caed34a69fbc1984c4803d87852b271
   def cube(self, N, L):
      self.coord = formation.cube(N, L)

   def piramide(self):
      coord = formation.piramide(self, self.num_of_clovers)
      return coord

   #Leader operations
   def setLeader(self, id):
      
      assert type(id)==int, "Input 'id' must be an integer."
      self.leader_id = id
   
   def followLeader():
      pass

   def launchSwarm(self):
      pass

if __name__ == "__main__":
   swarm = Swarm(6)
   N = swarm.num_of_clovers

   while not rospy.is_shutdown():
      menu()
      key= input("\n")
      if (key == str('1')):
         coord = swarm.takeoff_all()
         print("Drones coordinates: \n{}\n".format(coord))
         #rospy.sleep(2)
      elif (key == str('2')):
         if (N < 2):
            print("You need at least 2 clovers!\n")
         else:
            L = int(input("Insert the desired length: "))
            coord = swarm.line(N=N, L=L)
            print("Drones coordinates: \n{}\n".format(coord))
            #rospy.sleep(5)

      elif (key == str('3')):
         if (N < 3):
            print("You need at least 3 clovers!\n")
         else:
               #N = int(input("Number of drones "))
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
            L = int(input("Insert the desired side length: "))
            coord = swarm.square(N=N, type=type, L=L)
            print("Drones coordinates: \n{}\n".format(coord))
            #rospy.sleep(5)

      elif (key == str('5')):
         if (N < 8):
               print("You need at least 8 clovers!\n")
         else:
               #type = input("Insert full or empty: ")
               L = int(input("Insert the desired side length: "))
               coord = swarm.cube(N=N, L=L)
               print("Drones coordinates: \n{}\n".format(coord))
               #rospy.sleep(5)

      elif (key == str('6')):
         coord = swarm.sphere(N=N)
         print("Drones coordinates: \n{}\n".format(coord))
         #rospy.sleep(5)

      elif (key == str('7')):
         if (N < 3):
               print("You need at least 3 clovers!\n")
         else:
               # x0 = int(input("Insert initial x coordinate: "))
               # y0 = int(input("Insert initial y coordinate: "))
               # z0 = int(input("Insert the desired height: "))
               # L = int(input("Insert the desired side length: "))
               swarm.piramide()
               rospy.sleep(5)

      elif (key == str('o') or key == str('O')):
         r = int(input("Insert the desired ratio: "))
         xc = int(input("Insert the center x coordinate: "))
         yc = int(input("Insert the center y coordinate: "))
         coord = swarm.circle(N=N, xc=xc, yc=yc, r=r)
         print("Drones coordinates: \n{}\n".format(coord))
         #rospy.sleep(2)

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



