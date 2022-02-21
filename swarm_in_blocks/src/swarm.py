#!/usr/bin/python3

# ROS modedules
import rospy
from mavros_msgs import srv
from mavros_msgs.msg import State

# Clover services
from clover import srv
from std_srvs.srv import Trigger

# Other tools
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from multiprocessing import Process, Pool
import time
import sys
import os
import logging

# Local modules
import formation
import launch
import transform

#Menu 
def menu():
   print("Press")
   print("1 - takeoff all")
   print("2 - line formation")
   print("3 - triangle formation")
   print("4 - square formation")
   print("5 - cube formation")
   print("6 - sphere formation")
   print("7 - pyramid formation")
   print("O - circle formation")
   print("0 - initial position")
   print("L - land all")
   print("E - exit")

class SingleClover: 
#Create and call all servicers, subscribers and clover topics

   def __init__(self, name, id):
      self.name = name
      self.id = id
      self.init_coord = []

      self.current_state = State()

      # Configure clover services and topics
      self.configure()

   def stateCb(self, msg_cb):
      self.current_state = msg_cb

   def configure(self):

      logging.info("Waiting clover services...")

      self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)
      
      rospy.wait_for_service(f"{self.name}/get_telemetry")   
      self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
      
      rospy.wait_for_service(f"{self.name}/navigate")
      self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
      
      rospy.wait_for_service(f"{self.name}/navigate_global")
      self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
      
      rospy.wait_for_service(f"{self.name}/set_position")
      self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
      
      rospy.wait_for_service(f"{self.name}/set_velocity")
      self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
      
      rospy.wait_for_service(f"{self.name}/set_attitude")
      self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
      
      rospy.wait_for_service(f"{self.name}/set_rates")
      self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
      
      rospy.wait_for_service(f"{self.name}/land")
      self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger) 
   
   def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
      
      self.navnavigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

      while not rospy.is_shutdown():
         telem = self.get_telemetry(frame_id='navigate_target' + str(self.id))
         if np.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
               break
         rospy.sleep(0.2)

class Swarm:
   # Private functions
   def __init__(self, num_of_clovers):
      
      # Basic parameters of the swarm
      self.num_of_clovers = num_of_clovers
      self.swarm = []
      self.id_clover = []

      # Coordinates from the center of the formation
      self.form_pose = [0,0,0,0,0,0]

      # Initial formation
      self.init_formation_coords = []

      # Current formation name and coord in homogeneous line vector type 
      # Ex of self.curr_form_coords: [[x0,y0,z0,1],[x1,y1,z1,1], [x2,y2,z2,1], ...]
      self.curr_formation_name = ''
      self.curr_formation_coords = []

      # Desired formation
      self.des_formation_coord = []
      
      # Initial formation. By default on square formation with L = N//2 + 1.
      # (Ex: N=5 -> L=3)
      self.setFormation2D('line', self.num_of_clovers, self.num_of_clovers-1)
      self.init_formation_coords = self.des_formation_coord

      # Lists of formations. Can be used to plan a formation changing for formation that is hard to generate in real time.
      self.formation_list = []

      # Leader id of the swarm
      self.leader_id = None

   def __launchGazeboAndClovers(self):
      launch.spawnGazeboAndVehicles(self.num_of_clovers, self.init_formation_coords)
      time.sleep(5*self.num_of_clovers)
   
   # Create clover objects and append to clover object list
   def __createCloversObjects(self):
      for index in range(self.num_of_clovers):
         clover_object = SingleClover(f"clover{index}", index)
         clover_object.init_coord = self.init_formation_coords[index]
         self.swarm.append(clover_object)

   def setInitialFormation(self, formation_str, L):
      self.setFormation2D(formation_str, self.num_of_clovers, L)
      self.init_formation_coords = self.des_formation_coord

   # Start pipeline
   def startPlanning(self):
      pass

   def startSimulation(self, already_launched=True):

      # Launch Gazebo and clover. Wait some time to all get
      if not already_launched:
         print("Starting roscore, Gazebo and clovers")
         self.__launchGazeboAndClovers()

      # Create clover python objects
      print("Starting swarm node and listening to clover services.")
      rospy.init_node('swarm')
      self.__createCloversObjects()

   def startNavigation(self):
      pass   

   def applyFormation(self):
      coord = self.des_formation_coord
      for idx, clover in enumerate(self.swarm):
         x0 = 0 - clover.init_coord[0]
         y0 = 0 - clover.init_coord[1]
         clover.navigate(x=x0+coord[idx][0],y=y0+coord[idx][1],z=coord[idx][2])
      self.curr_formation_coords = coord

   #Preview formations
   def plot_preview(self, coord):
      plt.figure(figsize=(8, 8))
      plt.subplots_adjust(bottom = 0.2)
      plt.plot(coord[:,0],coord[:,1],'ro')
      plt.axis([-1,11,-1,11])
      plt.grid(True)
      plt.xticks(np.linspace(0,10,11))
      plt.yticks(np.linspace(0,10,11))
      posit = plt.axes([0.4, 0.1, 0.2, 0.05])
      button = Button(posit,'Confirm')
      #button.on_clicked(swarm.applyFormation)
      plt.show(block=False)

   def plot_preview_3d(self, coord):
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

   #Basic swarm operations
   def takeoff_all(self, z0=1):
      coord = np.empty((0,4))
      print("All drones taking off")
      for clover in self.swarm:
         point = [clover.init_coord[0], clover.init_coord[1], z0, 1]
         clover.navigate(x=0, y=0, z=z0, frame_id='map', auto_arm=True)
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      self.curr_formation_coords = coord

   def return_to_home(self):
      coord = np.empty((0,4))
      print("All drones returning")
      for clover in self.swarm:
         point = [clover.init_coord[0], clover.init_coord[1], 1, 1]
         clover.navigateWait(x=0, y=0, z=1)
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      self.curr_formation_coords = coord

   def land_all(self):
      coord = np.empty((0,4))
      print("All drones landing")
      for clover in self.swarm:
         clover.land()
         point = [clover.init_coord[0], clover.init_coord[1],0,1]
         coord = np.concatenate((coord,[point]))
      self.plot_preview(coord)
      self.curr_formation_coords = coord

   #Formations
   def setFormation2D(self, shape, N, L):
      if (shape=='line'):
         coord = formation.line(N, L)
      elif (shape=='full_square'):
         coord = formation.full_square(N, L)
      elif (shape=='empty_square'):
         coord = formation.empty_square(N, L)
      elif (shape=='circle'):
         coord = formation.circle(N, L)
      elif (shape=='triangle'):
         coord = formation.triangle(N, L)
      else:
         raise Exception('Formation input doesn\'t match any built-in formations')
      self.des_formation_coord = coord

   def setFormation3D(self, shape, N, L):
      if (shape=='cube'):
         coord = formation.cube(N, L)
      elif (shape=='sphere'):
         coord = formation.sphere(N, L)
      elif (shape=='pyramid'):
         coord = formation.pyramid(N, L)
      else:
         raise Exception('Formation input doesn\'t match any built-in formations')
      self.des_formation_coord = coord

   #Transformations
   def transformFormation(self, coord, sx, sy, sz, anglex, angley, anglez, tx, ty, tz):
      new_coord = transform.transformFormation(coord, sx, sy, sz, anglex, angley, anglez, tx, ty, tz)
      self.des_formation_coord = new_coord

   def scaleFormation(self, coord, sx, sy, sz):
      new_coord = transform.scaleFormation(coord, sx, sy, sz)
      self.des_formation_coord = new_coord

   def translateFormation(self, coord, tx, ty, tz):
      new_coord = transform.translateFormation(coord, tx, ty, tz)
      self.des_formation_coord = new_coord

   def rotateFormation(self, coord, anglex, angley, anglez):
      new_coord = transform.rotateFormation(coord, anglex, angley, anglez)
      self.des_formation_coord = new_coord

   #Leader operations
   def setLeader(self, id):
      assert type(id)==int, "Input 'id' must be an integer."
      self.leader_id = id
   
   def followLeader():
      pass

if __name__ == "__main__":

   #Menu 
   def menu():
      print("Press")
      print("1 - takeoff all")
      print("2 - line formation")
      print("3 - triangle formation")
      print("4 - square formation")
      print("5 - cube formation")
      print("6 - sphere formation")
      print("7 - pyramid formation")
      print("O - circle formation")
      print("0 - initial position")
      print("ap - apply formation")
      print("L - land all")
      print("E - exit")

   swarm = Swarm(4)
   swarm.startSimulation(already_launched=False)
   N = swarm.num_of_clovers
   #init_form = swarm.setInitialPosition()
   #swarm.launchGazeboAndClovers(init_form)


   while not rospy.is_shutdown():
      coord = swarm.des_formation_coord
      menu()
      key = input('\n')
      if (key == str('1')):
         swarm.takeoff_all()
         print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
         #rospy.sleep(2)

      elif (key == str('2')):
         if (N < 2):
            print("You need at least 2 clovers!\n")
         else:
            L = int(input("Insert the desired length: "))
            swarm.setFormation2D('line', N, L)
            print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
            #rospy.sleep(5)

      elif (key == str('3')):
         if (N < 3):
            print("You need at least 3 clovers!\n")
         else:
               L = int(input("Insert the desired side length: "))
               swarm.setFormation2D('triangle', N, L)
               print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
               #rospy.sleep(5)

      elif (key == str('4f') or key == str('4F')):
         if (N < 4):
            print("You need at least 4 clovers!\n")
         else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('full_square', N, L)
            print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
            #rospy.sleep(5)

      elif (key == str('4e') or key == str('4E')):
         if (N < 4):
            print("You need at least 4 clovers!\n")
         else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('empty_square', N, L)
            print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
            #rospy.sleep(5)

      elif (key == str('o') or key == str('O')):
         L = int(input("Insert the desired ratio: "))
         swarm.setFormation2D('circle', N, L)
         print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
         #rospy.sleep(2)

      elif (key == str('5')):
         if (N < 8):
               print("You need at least 8 clovers!\n")
         else:
               #type = input("Insert full or empty: ")
               L = int(input("Insert the desired side length: "))
               swarm.setFormation3D('cube', N, L)
               print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
               #rospy.sleep(5)

      elif (key == str('6')):
         L = int(input("Insert the desired ratio: "))
         swarm.setFormation3D('sphere', N, L)
         print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
         #rospy.sleep(5)

      elif (key == str('7')):
         if (N < 3):
               print("You need at least 3 clovers!\n")
         else:
               L = int(input("Insert the desired side length: "))
               swarm.setFormation3D('pyramid', N, L)
               print("Drones coordinates: \n{}\n".format(swarm.des_formation_coord))
               rospy.sleep(5)

      elif (key == str('0')):
         swarm.return_to_home()
         print("Drones coordinates: \n{}\n".format(swarm.curr_formation_coords))
         rospy.sleep(2)

      elif (key == str('l') or key == str('L')):
         swarm.land_all()
         print("Drones coordinates: \n{}\n".format(swarm.curr_formation_coords))
         rospy.sleep(5)

      elif (key == str('ms')):
         sx = int(input("Insert the x scale: "))
         sy = int(input("Insert the y scale: "))
         sz = int(input("Insert the z scale: "))
         swarm.scaleFormation(swarm.des_formation_coord, sx, sy, sz)
         swarm.plot_preview(swarm.des_formation_coord)
      
      elif (key == str('mr')):
         anglex = float(input("Insert the x angle: "))
         angley = float(input("Insert the y angle: "))
         anglez = float(input("Insert the z angle: "))
         swarm.rotateFormation(swarm.des_formation_coord, anglex, angley, anglez)
         swarm.plot_preview(swarm.des_formation_coord)

      elif (key == str('mt')):
         tx = int(input("Insert the x translation: "))
         ty = int(input("Insert the y translation: "))
         tz = int(input("Insert the z translation: "))
         swarm.translateFormation(swarm.des_formation_coord, tx, ty, tz)
         swarm.plot_preview(swarm.des_formation_coord)

      elif (key == str('ciranda')):
         ang=0
         while(ang < 4*np.pi):
            swarm.rotateFormation(swarm.des_formation_coord, 0, 0, ang)
            rospy.sleep(2)
            swarm.applyFormation(swarm.des_formation_coord)
            ang += 0.2
            rospy.sleep(3)

      elif (key == str('ap')):
         swarm.applyFormation()

      elif (key == str('e') or key == str('E')):
         break