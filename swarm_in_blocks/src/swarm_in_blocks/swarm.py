#!/usr/bin/python3

# ROS modedules
from re import A
from matplotlib.pyplot import connect
import rospy
from mavros_msgs import srv

# Clover services
from clover import srv
from std_srvs.srv import Trigger

# Other tools
import numpy as np
from threading import Thread
from multiprocessing import Process
import time
import sys
import os
import json
import traceback
import logging
import math
import random

# Local modules
sys.path.insert(0, os.path.dirname(__file__))
import formation
import launch
import transform
import alphabet
import plot
import formation3D
from swarm_api_publisher import SwarmPublisher
from swarm_checker.msg import SwarmState

class SingleClover: 
#Create and call all servicers, subscribers and clover topics
   def __init__(self, name, clover_id):
      self.name = name
      self.id = clover_id
      self.init_coord = []

      # Configure clover services and topics
      self.configure()

   def configure(self):

      logging.debug("Waiting clover services...")
      rospy.loginfo("Waiting clover services...")

      rospy.wait_for_service(f"{self.name}/get_telemetry", timeout=1)
      self.get_telemetry = rospy.ServiceProxy(f"{self.name}/get_telemetry", srv.GetTelemetry)
      
      rospy.wait_for_service(f"{self.name}/navigate", timeout=1)
      self.navigate = rospy.ServiceProxy(f"{self.name}/navigate", srv.Navigate)
      
      rospy.wait_for_service(f"{self.name}/navigate_global", timeout=1)
      self.navigate_global = rospy.ServiceProxy(f"{self.name}/navigate_global", srv.NavigateGlobal)
      
      rospy.wait_for_service(f"{self.name}/set_position", timeout=1)
      self.set_position = rospy.ServiceProxy(f"{self.name}/set_position", srv.SetPosition)
      
      rospy.wait_for_service(f"{self.name}/set_velocity", timeout=1)
      self.set_velocity = rospy.ServiceProxy(f"{self.name}/set_velocity", srv.SetVelocity)
      
      rospy.wait_for_service(f"{self.name}/set_attitude", timeout=1)
      self.set_attitude = rospy.ServiceProxy(f"{self.name}/set_attitude", srv.SetAttitude)
      
      rospy.wait_for_service(f"{self.name}/set_rates", timeout=1)
      self.set_rates = rospy.ServiceProxy(f"{self.name}/set_rates", srv.SetRates)
      
      rospy.wait_for_service(f"{self.name}/land", timeout=1)
      self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)

      rospy.wait_for_service(f"{self.name}/led/set_effect", timeout=1)
      self.set_effect = rospy.ServiceProxy(f"{self.name}/led/set_effect", srv.SetLEDEffect)
   
   def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
      
      self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

      while not rospy.is_shutdown():
         telem = self.get_telemetry(frame_id='navigate_target' + str(self.id))
         if np.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
               break
         rospy.sleep(0.2)

class Swarm:
   # Private functions
   def __init__(self, num_of_clovers=None, swarm_name=None):

      # Set logging level if it doesn't exist
      logging.basicConfig(level=logging.INFO)
      # logging.getLogger('rosout')

      # Basic parameters of the swarm
      self.swarm = []
      self.swarm_state = SwarmState()

      # Swarm infos
      self.num_of_clovers = None
      if num_of_clovers != None:
         if num_of_clovers == 0:
            logging.error('Input num_of_clover need to be greater than zero.')
         else:
            self.num_of_clovers = num_of_clovers
      
      self.all_of_clovers = 0
      self.connected_clovers = 0
      self.armed_clovers = 0
      self.offboard_mode_clovers = 0
      # Ids
      self.all_clovers_ids = []
      self.connected_ids = []

      # Configure swarm_name
      if swarm_name is None:
         self.swarm_name = "CloverBlockingSquad!"
      else:
         self.swarm_name = swarm_name
      
      # Initial formation
      self.init_formation_name = ''
      self.init_formation_coords = []

      # Current formation name and coord in homogeneous line vector type 
      # Ex of self.curr_form_coords: [[x0,y0,z0,1],[x1,y1,z1,1], [x2,y2,z2,1], ...]
      self.curr_formation_name = ''
      self.curr_formation_coords = np.empty((0,4))
      
      # X Y Z Roll Pitch Yaw format
      self.curr_formation_pose = np.array([0,0,0])
      self.des_formation_pose = np.array([0,0,0])

      # Desired formation coords
      self.des_formation_name = ''
      self.des_formation_coords = []

      # Led matrix
      self.des_led_effects_name = ''
      self.des_led_effects = []

      # Mode selected upon start
      self.mode = ''
      
      # Lists of formations. Can be used to plan a formation changing for formation that is hard to generate in real time.
      self.op_num = 0
      self.formation_list = {}

      # Leader id of the swarm
      self.leader_id = None

      # Private atributes for internal class organization
      self.__mesh = None
      self.__pcd = None
      self.__meshzoo_names = ['christ_the_redeemer','motherland_calls']

   def __launchGazeboAndClovers(self):
      launch.simulation(self.num_of_clovers, self.init_formation_coords)
   
   # Create clover objects and append to clover object list
   def __createCloversObjects(self):
      for idx, clover_id in enumerate(self.connected_ids):
         clover_object = SingleClover(f"clover{clover_id}", clover_id)
         clover_object.init_coord = self.init_formation_coords[idx]
         self.swarm.append(clover_object)
   
   def __subscribeSwarmChecker(self):
      rospy.loginfo("Trying to connect to SwarmChecker node...")
      try:
         first_msg = rospy.wait_for_message("swarm_checker/state", SwarmState, timeout=5)
         self.swarm_checker_sub = rospy.Subscriber("swarm_checker/state", SwarmState, self.__swarmStateCallback)
      except Exception as e:
         rospy.logerr(e.__str__() + "\n Have you launched the simulation or the  swarm_checker node?")
         exit()
         
      rospy.loginfo("SwarmApi connected to SwarmChecker.")

      # Configure first time
      self.swarm_state = first_msg
      self.all_of_clovers = self.swarm_state.all_clovers
      self.connected_clovers =self.swarm_state.connected_clovers
      self.armed_clovers = self.swarm_state.armed_clovers

      self.all_clovers_ids = self.swarm_state.all_clovers_ids
      self.connected_ids = self.swarm_state.connected_ids

   def __swarmStateCallback(self, msg):
      self.swarm_state = msg
      self.all_of_clovers = self.swarm_state.all_clovers
      self.connected_clovers =self.swarm_state.connected_clovers
      self.armed_clovers = self.swarm_state.armed_clovers

      self.all_clovers_ids = self.swarm_state.all_clovers_ids
      self.connected_ids = self.swarm_state.connected_ids

   def setInitialFormation(self, formation_str, L):
      self.setFormation2D(formation_str, self.num_of_clovers, L)
      self.init_formation_coords = self.des_formation_coords

   def getInitialFormation(self):
      coords = np.zeros((self.num_of_clovers, 4))
      coords[:] = np.NaN

      for idx, clover_id in enumerate(self.connected_ids):
         try:
            x = rospy.get_param(f"/clover{clover_id}/initial_pose/x")
            y = rospy.get_param(f"/clover{clover_id}/initial_pose/y")
            z = rospy.get_param(f"/clover{clover_id}/initial_pose/z")
            coords[idx] = [x, y, z, 1]
         except Exception:
            print(traceback.format_exc())
            continue
      
      # See if there is a clover that wasnt able to retrieve it initial pose
      initial_pose_failed = np.argwhere(np.isnan(coords))
      initial_pose_failed = initial_pose_failed[:,0]
      initial_pose_failed = np.unique(initial_pose_failed)

      # Update initial_formation
      if not initial_pose_failed.size:
         self.init_formation_coords = coords
         self.des_formation_coords = self.init_formation_coords

      # Log the result
      logging.debug(f"{self.num_of_clovers} retrieved their initial pose.")
      for clover_id in initial_pose_failed:
         logging.debug(f"Failed to get initial pose from clover {clover_id}")
      logging.debug(f"Sucessfully got initial pose from {self.num_of_clovers-len(initial_pose_failed)} clovers, failed with {len(initial_pose_failed)} clovers.")

   # Start pipeline
   # Planning mode - Allows just to plot the formations preview and save its coordinates, don't use simulator
   def startPlanning(self):
      logging.debug("Starting planning mode...")
      # rospy.loginfo("Starting planning mode...")
      self.mode = 'Planning'
      # See if initial formation is [], then set default inital formation
      if not self.init_formation_coords:
         # Initial formation for 3 or less clovers. By default on line formation.
         if self.num_of_clovers < 4:
            self.setInitialFormation('line', self.num_of_clovers-1)
         # Initial formation for more than 3 clovers. By default on square formation with L = N//2 + 1.
         # (Ex: N=5 -> L=3)
         else:
            self.setInitialFormation('full_square', int(np.sqrt(self.num_of_clovers)))
      plot.plot_init(self)

   # Simulation mode - Allows to simulate the clovers on Gazebo and use all the developed features
   def startSimulation(self, launch=False):

      if self.mode == 'simulation':
         logging.debug("Simulation has already started.")
         rospy.loginfo("Simulation has already started.")
         return
      
      logging.debug("Starting simulation mode...")
      rospy.loginfo("Starting simulation mode...")
      self.mode = 'Simulation'
      
      # Launch Gazebo and clover. Wait some time to all get
      if launch:
         if self.num_of_clovers == None:
            raise Exception("Attribute num_of_clovers wasn't specified to launch the simulation properly.")
         # See if initial formation is [], then set default inital formation
         if not self.init_formation_coords:
            # Initial formation for 3 or less clovers. By default on line formation.
            if self.num_of_clovers < 4:
               self.setInitialFormation('line', self.num_of_clovers-1)
            # Initial formation for more than 3 clovers. By default on square formation with L = N//2 + 1.
            # (Ex: N=5 -> L=3)
            else:
               self.setInitialFormation('full_square', int(np.sqrt(self.num_of_clovers)))
         logging.debug("Starting roscore, Gazebo and clovers...")
         rospy.loginfo("Starting roscore, Gazebo and clovers...")
         self.__launchGazeboAndClovers()
      
      # Init rosnode
      rospy.init_node('swarm_api')

      # Connecting to swarm_checker node
      self.__subscribeSwarmChecker()

      # Configure num_of_clovers
      if self.num_of_clovers is None:
         self.num_of_clovers = self.connected_clovers
         if self.num_of_clovers == 0:
            raise Exception("Failed to init swarm object. Have you launched the simulation or the navigation?")

      if not launch:
         # self.checkClovers()
         self.getInitialFormation()

      # Create clover python objects
      logging.debug("Starting swarm node and listening to clover services...")
      rospy.loginfo("Starting swarm node and listening to clover services...")

      # Create clover objects for Swarm class
      self.__createCloversObjects()

      # All done. Updating current formation
      self.status = 'Connected'
      self.curr_formation_coords = self.des_formation_coords

      # Set up SwarmPublisher to publish swarm informations
      swarm_pub = SwarmPublisher()
      Thread(target=swarm_pub.publishStatusLoop, args=(self,)).start()
      Thread(target=swarm_pub.publishAssetsLoop, args=(self,)).start()

      logging.debug("Started simulation.")
      rospy.loginfo("Started simulation.")

   # Navigation mode - Mode for those who want to fly with a real swarm
   def startNavigation(self):
      self.mode = 'Navigation'   

   # Basic swarm operations
   def takeOffAll(self, z=1, speed=0.5):
      logging.debug(f"{self.num_of_clovers} drones taking off")
      rospy.loginfo(f"{self.num_of_clovers} drones taking off")
      self.des_formation_coords = self.init_formation_coords
      self.des_formation_coords[:,2] = z
      self.des_formation_pose = np.array([0,0,z])
      
      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]  
         thrd = Thread(target=clover.navigateWait, kwargs=dict(x=x, y=y, z=z, tolerance=0.2, speed=speed, auto_arm=True))
         threads.append(thrd)
      
      for thrd in threads:
         thrd.start()
      
      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords =  self.des_formation_coords
      self.curr_formation_pose = self.des_formation_pose

   def landAll(self):
      coord = np.empty((0,4))
      logging.debug(f"{self.num_of_clovers} drones landing")
      rospy.loginfo(f"{self.num_of_clovers} drones landing")
      for clover in self.swarm:
         clover.land()
         point = [clover.init_coord[0], clover.init_coord[1],0,1]
         coord = np.concatenate((coord,[point]))
      self.curr_formation_coords = coord

      self.des_formation_coords = self.init_formation_coords
      self.des_formation_coords[:,2] = 0
      self.des_formation_pose = np.array([0,0,0])

      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]  
         thrd = Thread(target=clover.land)
         threads.append(thrd)
      
      for thrd in threads:
         thrd.start()

      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords = self.des_formation_coords
      self.curr_formation_pose = self.des_formation_pose

   def returnToHome(self):
      logging.debug(f"{self.num_of_clovers} drones returning")
      rospy.loginfo(f"{self.num_of_clovers} drones returning")
      self.des_formation_coords = self.init_formation_coords
      self.des_formation_pose = np.array([0, 0, 0])
      self.applyFormation()   

   def returnAndLand(self):
      logging.debug("Return to home...")
      rospy.loginfo("Return to home...")
      self.returnToHome()
      logging.debug("Landing...")
      rospy.loginfo("Landing...")
      self.landAll()

   def applyFormation(self, speed=1, tolerance=0.2, wait=True):
      logging.debug(f"Applying formation to {self.num_of_clovers}")
      rospy.loginfo(f"Applying formation to {self.num_of_clovers}")
      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]
         if wait:
            thrd = Thread(target=clover.navigateWait, kwargs=dict(x=x,y=y,z=z, speed=speed, tolerance=0.2))
         else:
            thrd = Thread(target=clover.navigate, kwargs=dict(x=x,y=y,z=z, speed=speed))
         threads.append(thrd)
      
      # Start all threads with the minimum latency possible
      for thrd in threads:
         thrd.start()
      
      # Wait for all threads
      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords =  self.des_formation_coords
      self.curr_formation_pose = self.des_formation_pose

   def plot_preview(self, plot_type='2D'):
      if plot_type == '2D':
         plot.create_swarm_preview(self, self.des_formation_coords, preview_type='2D')
      elif plot_type == '3D':
         plot.create_swarm_preview(self, self.des_formation_coords, preview_type='3D')
      else:
         raise Exception("Type isn't allowed, select between '2D' or '3D'")

   # LED Operations
   def ledAll(self, effect, red, green, blue):
      logging.debug(f"{self.num_of_clovers} setting all drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting all drones led")

      threads = []
      for idx, clover in enumerate(self.swarm):
         thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=red, g=green, b=blue))
         thrd.start()
         threads.append(thrd)
      
      for thrd in threads:
         thrd.join(timeout=1)

   def ledRandom(self, effect):

      logging.debug(f"{self.num_of_clovers} setting all drones random led color")
      rospy.loginfo(f"{self.num_of_clovers} setting all drones random led color")

      threads = []
      for idx, clover in enumerate(self.swarm):
         thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=random.randint(0, 255), g=random.randint(0, 255), b=random.randint(0, 255)))
         thrd.start()
         threads.append(thrd)
      
      for thrd in threads:
         thrd.join(timeout=1)

   def ledEven(self, effect, red, green, blue):
      logging.debug(f"{self.num_of_clovers} setting odd number drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting odd number drones led")

      threads = []
      for idx, clover in enumerate(self.swarm):
         if (idx%2 == 0):
            thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=red, g=green, b=blue))
            thrd.start()
            threads.append(thrd)
         else:
            continue   
      
      for thrd in threads:
         thrd.join(timeout=1)

   def ledOdd(self, effect, red, green, blue):
      logging.debug(f"{self.num_of_clovers} setting odd number drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting odd number drones led")

      threads = []
      for idx, clover in enumerate(self.swarm):
         if (idx%2 != 0):
            thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=red, g=green, b=blue))
            thrd.start()
            threads.append(thrd)
         else:
            continue
               
      
      for thrd in threads:
         thrd.join(timeout=1)
   
   def support_led_formation_2D(self, shape, N):
      color = np.empty((0,3), dtype=int)

      if(shape == "square"):
         side = 2

      if(shape == "triangle"):
         side = 2

      if(shape == "pyramid"):
         side = 3

      if((shape == "circle") & ((N%2) == 0)):
         side = 2

      if((shape == "circle") & ((N%2) != 0)):
         side = 3

      if(shape == "cube"):
         side = int(np.cbrt(N))

      for i in range (side):
            red = int(input(f"Insert the red color {i+1} (0-255): "))
            green = int(input(f"Insert the green color {i+1} (0-255): "))
            blue = int(input(f"Insert the blue color {i+1} (0-255): "))
            list_color = [red, green, blue]
            color = np.concatenate((color,[list_color]))
      
      return color

   def ledFormation2D(self, effect, shape, L, N):
      logging.debug(f"{self.num_of_clovers} setting odd number drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting odd number drones led")
      threads = []
      color = self.support_led_formation_2D(shape, N)
      n = np.cbrt(N)
      z = 1
      coord = self.des_formation_coords
      lista = [None] * N
      listaz = [None] * int(n)
      lista0 = [None] * N
      lista1 = [None] * N
      lista2 = [None] * N
      h = (np.sqrt(3)*L)/2
      a = 0
      b = 1
      c = 2
      
      print(coord)
      for i in range(0, int(n)):
         listaz[i] = z
         z = z + L/(n-1)

      for i in range(0, N):
         lista0[i] = a
         lista1[i] = b
         lista2[i] = c
         a += 3
         b += 3        
         c += 3

      for idx, clover in enumerate(self.swarm):
         if(shape == "triangle"):
            if(round(np.sqrt((((coord[idx][1])**2) + ((coord[idx][0])**2)))) == round(h*(2/3))):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
               print("teste1")
            
            else:
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
               print("teste2")
         
         if(shape == "square"):
            if(math.sqrt((((coord[idx][1])**2) + ((coord[idx][0])**2))) == (math.sqrt(((L/2)**2)+((L/2)**2)))):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
               
            else:
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
               

         if((shape == "circle") and ((N%2) == 0)):        
            if(idx%2 == 0):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2])) 
            
            else:
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))

         if((shape == "circle") and ((N%2) != 0)):        
            for i in range(0, N):
               if(idx == lista0[i]):
                  lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
            
               if(idx == lista1[i]):
                  lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
            
               if(idx == lista2[i]):
                  lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[2][0], g=color[2][1], b=color[2][2]))      
         
         if(shape == "cube"):
            for i in range(0, int(n)):
               if (coord[idx][2] == float(listaz[i])):
                  lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[i][0], g=color[i][1], b=color[i][2]))
                  
         if(shape == "pyramid"):
            
            if((idx-1)%3==0):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
            
            
            if((idx-2)%3==0):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
            
        
            if((idx%3)==0):
               lista[idx] = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[2][0], g=color[2][1], b=color[2][2]))


      print(lista)

      for idx, clover in enumerate(self.swarm):
         thrd = lista[idx] 
         thrd.start()
         threads.append(thrd) 
      
      for thrd in threads:
         thrd.join(timeout=1)

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
      self.des_formation_coords = coord

      # Translate formation for current formation pose
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      self.des_formation_coords = transform.translateFormation(self.des_formation_coords, tx, ty, tz)

      # Update formation name
      self.des_formation_name = shape
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def setFormation3D(self, shape, N, L):
      if (shape=='cube'):
         coord = formation.cube(N, L)
      elif (shape=='sphere'):
         coord = formation.sphere(N, L)
      elif (shape=='pyramid'):
         coord = formation.pyramid(N, L)
      else:
         raise Exception('Formation input doesn\'t match any built-in formations')
      self.des_formation_coords = coord

      # Translate formation for current formation pose
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      self.des_formation_coords = transform.translateFormation(self.des_formation_coords, tx, ty, tz)
      
      # Update formation name
      self.des_formation_name = shape
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def setAlphabet(self, user_input, N):
      z = 1
      if(user_input=="SWARM_S" or user_input=="swarm_s"):
         self.des_formation_coords = alphabet.Letters(user_input, N)
      
      else:
         self.des_formation_coords = alphabet.Word(user_input, N)
      
      for idx in range(self.num_of_clovers):
         if(idx%2==0):
            z = 2
            if((idx)%5==0 and (idx>5)):
               z += 1
               if(idx%2==0):
                  z = 2
         self.des_formation_coords[idx][2] = z
      
      # Translate formation for current formation pose
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      self.des_formation_coords = transform.translateFormation(self.des_formation_coords, tx, ty, tz)

      # Update formation name
      self.des_formation_name = "Text: '{}'".format(user_input)
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def setFormation3DfromMesh(self, model_path):
      self.des_formation_coords,self.__mesh,self.__pcd = formation3D.formation3DFromMesh(model_path, self.num_of_clovers)

   def setFormation3DFromMeshZoo(self, mesh_name):

      if mesh_name.lower() not in self.__meshzoo_names:
         raise Exception("Input mesh name is not on mesh zoo. The name is correct?")
      #TODO

   def visualizePointCloud(self):
      formation3D.visualizePointCloud(self.__pcd)

   def visualizeMesh(self):
      formation3D.visualizeMesh(self.__mesh)
   
   def loadFormation(self):
      # with open(os.path.dirname(os.path.abspath(__file__))+'/saved_files/last_formation.npy', 'rb') as f:
      #    self.des_formation_coords = np.load(f)
      # print(self.des_formation_coords)

      with open(os.path.dirname(os.path.abspath(__file__))+'/saved_files/last_formation.json') as json_file:
         loaded_list = json.load(json_file)
      for i in loaded_list:
        loaded_list[i]['coord'] = np.array(loaded_list[i]['coord'])
      name = loaded_list['formation {}'.format(len(loaded_list)-1)]['name']
      coord = loaded_list['formation {}'.format(len(loaded_list)-1)]['coord']

      # Update formation name
      loaded_formation_coords = coord
      loaded_formation_name = name
      self.des_formation_coords = coord
      self.des_formation_name = name
      #self.formation_list['formation {}'.format(self.op_num)] = {'name':loaded_formation_name, 'coord':loaded_formation_coords}
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   #Transformations
   # def transformFormation(self, sx, sy, sz, anglex, angley, anglez, tx, ty, tz):
   #    new_coord = transform.transformFormation(self.des_formation_coords, sx, sy, sz, anglex, angley, anglez, tx, ty, tz)
   #    self.des_formation_coords = new_coord
   #    self.des_formation_name = 'transform'
   #    self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
   #    self.op_num += 1

   def scaleFormation(self, sx, sy, sz):
      # Get x, y, z of current formation
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      # Translate back to the origin 
      origin_coords = transform.translateFormation(self.des_formation_coords, -tx, -ty, -tz)
      # Scale formation
      origin_coords = transform.scaleFormation(origin_coords, sx, sy, sz)
      # Translate back to the current pose
      self.des_formation_coords = transform.translateFormation(origin_coords, tx, ty, tz)
      # Update formation pose (stays the same in this case)
      self.des_formation_pose = np.array([self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]])
      self.des_formation_name = 'scale'
      # Append to formation list
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def translateFormation(self, tx, ty, tz):
      # Translate formation
      self.des_formation_coords = transform.translateFormation(self.des_formation_coords, tx, ty, tz)
      # Update formation pose 
      self.des_formation_pose = np.array([self.des_formation_pose[0]+tx, self.des_formation_pose[1]+ty, self.des_formation_pose[2]+tz])
      # Update formation name
      self.des_formation_name = 'translate'
      # Append to formation list
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def rotateFormation(self, anglex_deg, angley_deg, anglez_deg):
      anglex_rad = anglex_deg*np.pi/180
      angley_rad = angley_deg*np.pi/180
      anglez_rad = anglez_deg*np.pi/180
      # Get x, y, z of current formation
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      # Translate back to the origin 
      origin_coords = transform.translateFormation(self.des_formation_coords, -tx, -ty, -tz)
      # Rotate formation on the origin
      origin_coords = transform.rotateFormation(origin_coords, anglex_rad, angley_rad, anglez_rad)
      # Translate back to the current pose
      self.des_formation_coords = transform.translateFormation(origin_coords, tx, ty, tz)
      # Update formation pose (stays the same in this case)
      self.des_formation_pose = np.array([self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]])
      # Update formation name
      self.des_formation_name = 'rotate'
      # Append to formation list
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   #Leader operations
   def setChief(self, id):
      assert type(id)==int, "Input 'id' must be an integer."
      self.leader_id = id
   
   def followChiefThread(self):
      pass

   def followChief(self):
      self.stop_follow_chief = True

      if self.leader_id is None:
         pass
      
      # instanciate a process
      self.follow_chief_prcs = Process(target=self.followChiefThread)
      self.follow_chief_prcs.start()

      # terminate process when stopFollowingChief method is called
      while self.follow_chief_prcs.is_alive():
         if self.stop_follow_chief:
            self.follow_chief_prcs.terminate()

   def stopFollowingChief(self):
      self.stop_follow_chief = True