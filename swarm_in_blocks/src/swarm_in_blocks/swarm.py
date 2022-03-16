
#!/usr/bin/python3

# ROS modedules
from matplotlib.pyplot import connect
import rospy
import rosnode
from mavros_msgs import srv
from mavros_msgs.msg import State

# Clover services
from clover import srv
from std_srvs.srv import Trigger

# Other tools
import numpy as np
from threading import Thread
import time
import sys
import os
import traceback
import logging
import math

# Local modules
sys.path.insert(0, os.path.dirname(__file__))
import formation
import launch
import transform
import alphabet
import plot
import formation3D
from swarm_publisher import SwarmPublisher

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
      self.connected = msg_cb.connected
      self.armed = msg_cb.armed
      self.mode = msg_cb.mode

   def configure(self):

      logging.debug("Waiting clover services...")
      rospy.loginfo("Waiting clover services...")

      self.state = rospy.Subscriber(f"{self.name}/mavros/state", State, self.stateCb, queue_size=10)
      
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
      # logging.basicConfig(level=logging.INFO)
      logging.getLogger('rosout')
      
      # Basic parameters of the swarm
      # Configure swarm_name
      if swarm_name is None:
         self.swarm_name = "CloverBlockingSquad!"
      else:
         self.swarm_name = swarm_name
      
      # Configure num_of_clovers
      if num_of_clovers is None:
         self.num_of_clovers = self.__checkNumOfClovers()
         if self.num_of_clovers == 0:
            raise Exception("Failed to init swarm object. Have you launched the swarm?")
      else:
         self.num_of_clovers = num_of_clovers
      self.swarm = []
      self.id_clover = []

      # Swarm infos
      self.connected_clovers = 0
      self.armed_clovers = 0
      self.offboard_mode_clovers = 0

      # Initial formation
      self.init_formation_name = ''
      self.init_formation_coords = []

      # Current formation name and coord in homogeneous line vector type 
      # Ex of self.curr_form_coords: [[x0,y0,z0,1],[x1,y1,z1,1], [x2,y2,z2,1], ...]
      self.curr_formation_name = ''
      self.curr_formation_coords = []
      
      # X Y Z Roll Pitch Yaw format
      self.curr_formation_pose = np.array([0,0,0])
      self.des_formation_pose = np.array([0,0,0])

      # Desired formation
      self.des_formation_name = ''
      self.des_formation_coords = []

      # Led matrix
      self.led_effects_name = ''
      self.led_effects = []

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
      launch.spawnGazeboAndVehicles(self.num_of_clovers, self.init_formation_coords)
   
   # Create clover objects and append to clover object list
   def __createCloversObjects(self):
      for index in range(self.num_of_clovers):
         clover_object = SingleClover(f"clover{index}", index)
         clover_object.init_coord = self.init_formation_coords[index]
         self.swarm.append(clover_object)
   
   def __checkNumOfClovers(self):
      node_list = rosnode.get_node_names()

      num_of_clovers = 0
      for node in node_list:
         if ('clover' in node) and ('mavros' in node):
            num_of_clovers += 1
      
      return num_of_clovers
   
   def __checkStatusOfClovers(self):
      connected_clovers = 0
      offboard_mode_clovers = 0
      armed_clovers = 0
      
      for clover in self.swarm:
         connected = clover.connected
         mode = clover.mode
         armed = clover.armed

         if connected:
            connected_clovers+=1
         if mode.lower() == 'offboard':
            offboard_mode_clovers+=1
         if armed:
            armed_clovers+=1

      self.connected_clovers = connected_clovers
      self.armed_clovers = armed_clovers
      self.offboard_mode_clovers = offboard_mode_clovers
   
   def __checkStatusOfCloversLoop(self):
      rate = rospy.Rate(5)
      while not rospy.is_shutdown():
         self.__checkStatusOfClovers()
         rate.sleep()

   def setInitialFormation(self, formation_str, L):
      self.setFormation2D(formation_str, self.num_of_clovers, L)
      self.init_formation_coords = self.des_formation_coords

   def getInitialFormation(self):
      coords = np.zeros((self.num_of_clovers, 4))
      coords[:] = np.NaN

      for idx in range(self.num_of_clovers):
         try:
            x = rospy.get_param(f"/clover{idx}/initial_pose/x")
            y = rospy.get_param(f"/clover{idx}/initial_pose/y")
            z = rospy.get_param(f"/clover{idx}/initial_pose/z")
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
      if not initial_pose_failed.size:
         logging.debug(f"{self.num_of_clovers} retrieved their initial pose.")
         logging.debug(f"Sucessfully got initial pose from {self.num_of_clovers-len(initial_pose_failed)} clovers, failed {len(initial_pose_failed)} clovers.")
      else:
         logging.debug(f"Initial Formation: {self.num_of_clovers} retrieved their initial pose.")
         for clover_id in initial_pose_failed:
            logging.debug(f"Initial Formation: failed to get initial pose from clover {clover_id}")
         logging.debug(f"Initial formation: sucessfully got initial pose from {self.num_of_clovers-len(initial_pose_failed)} clovers, failed {len(initial_pose_failed)} clovers.")

   # Start pipeline
   # Planning mode - Allows just to plot the formations preview and save its coordinates, don't use simulator
   def startPlanning(self):
      logging.debug("Starting planning mode...")
      rospy.loginfo("Starting planning mode...")
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
      
      if not launch:
         # self.checkClovers()
         self.getInitialFormation()

      # Create clover python objects
      logging.debug("Starting swarm node and listening to clover services...")
      rospy.loginfo("Starting swarm node and listening to clover services...")

      # Init rosnode
      rospy.init_node('swarm_api')
      self.__createCloversObjects()

      # Check clovers
      self.__checkStatusOfClovers()

      # All done. Updating current formation
      self.status = 'Connected'
      self.curr_formation_coords = self.des_formation_coords

      # Set up SwarmPublisher to publish swarm informations
      swarm_pub = SwarmPublisher()
      Thread(target=self.__checkStatusOfCloversLoop).start()
      Thread(target=swarm_pub.publishStatusLoop, args=(self,)).start()
      Thread(target=swarm_pub.publishAssetsLoop, args=(self,)).start()
      # rospy.Timer(rospy.Duration(1.0),
                  # swarm_pub.publishSwarmStatus, args=(self.swarm_name, self.status, self.mode, self.connected_clovers))
      
      # rospy.Timer(rospy.Duration(0.1),
                  # swarm_pub.publishSwarmAssets, args=(self.curr_formation_name, self.curr_formation_pose, self.curr_formation_coords))
      logging.debug("Started simulation.")
      rospy.loginfo("Started simulation.")

   # Navigation mode - Mode for those who want to fly with a real swarm
   def startNavigation(self):
      self.mode = 'Navigation'   

   # Basic swarm operations
   def takeOffAll(self, z=1):
      logging.debug(f"{self.num_of_clovers} drones taking off")
      rospy.loginfo(f"{self.num_of_clovers} drones taking off")
      self.des_formation_coords = self.init_formation_coords
      self.des_formation_coords[:,2] = z
      
      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]  
         thrd = Thread(target=clover.navigateWait, kwargs=dict(x=x, y=y, z=z, tolerance=0.05, auto_arm=True))
         thrd.start()
         threads.append(thrd)
      
      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords =  self.des_formation_coords
   
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

      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]  
         thrd = Thread(target=clover.land, kwargs=dict(x=x,y=y,z=z))
         thrd.start()
         threads.append(thrd)
      
      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords =  self.des_formation_coords

   def returnToHome(self):
      logging.debug(f"{self.num_of_clovers} drones returning")
      rospy.loginfo(f"{self.num_of_clovers} drones returning")
      self.des_formation_coords = self.init_formation_coords
      self.applyFormation()   

   def returnAndLand(self):
      logging.debug("Return to home...")
      rospy.loginfo("Return to home...")
      self.returnToHome()
      logging.debug("Landing...")
      rospy.loginfo("Landing...")
      self.landAll()

   def applyFormation(self):
      logging.debug(f"Applying formation to {self.num_of_clovers}")
      rospy.loginfo(f"Applying formation to {self.num_of_clovers}")
      threads = []
      for idx, clover in enumerate(self.swarm):
         x = self.des_formation_coords[idx][0] - clover.init_coord[0]
         y = self.des_formation_coords[idx][1] - clover.init_coord[1]
         z = self.des_formation_coords[idx][2]  
         thrd = Thread(target=clover.navigateWait, kwargs=dict(x=x,y=y,z=z))
         threads.append(thrd)
      
      # Start all threads with the minimum latency possible
      for thrd in threads:
         thrd.start()
      
      # Wait for all threads
      for thrd in threads:
         thrd.join()
      
      self.curr_formation_coords =  self.des_formation_coords

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
         if idx%2 != 0:
            thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=red, g=green, b=blue))
            thrd.start()
            threads.append(thrd)
         else:
            continue
               
      
      for thrd in threads:
         thrd.join(timeout=1)

   def led_One_by_One(self):
      logging.debug(f"{self.num_of_clovers} setting odd number drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting odd number drones led")

      threads = []
      for idx, clover in enumerate(self.swarm):

         effect = str(input("input led effect: "))
         red = int(input("Insert the red color (0-255): "))
         green = int(input("Insert the green color (0-255): "))
         blue = int(input("Insert the blue color (0-255): "))

         thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=red, g=green, b=blue))
         thrd.start()
         threads.append(thrd)

      for thrd in threads:
         thrd.join(timeout=1)
   
   def support_led_formation_2D(self, shape):
      color = np.empty((0,3))
      
      if(shape == "square" or shape == "circle"):
         side = 2

      if(shape == "triangle"):
         side = 3

      for i in range (side):
            red = int(input(f"Insert the red color {i+1} (0-255): "))
            green = int(input(f"Insert the green color {i+1} (0-255): "))
            blue = int(input(f"Insert the blue color {i+1} (0-255): "))
            list_color = [red, green, blue]
            color = np.concatenate((color,[list_color]))
      
      return color

   def ledFormation2D(self, effect, str, L):
      logging.debug(f"{self.num_of_clovers} setting odd number drones led")
      rospy.loginfo(f"{self.num_of_clovers} setting odd number drones led")
      threads = []
      color = self.support_led_formation_2D(str)

      for idx, clover in enumerate(self.swarm):
         if(str == "triangle"):
            coord = self.des_formation_coords
            if(((L/2) - coord[idx][1]>=0) and (coord[idx][0]>0)):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
            
            elif(((L/2) - coord[idx][1]<0) and (coord[idx][0]>0)):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
            
            elif(coord[idx][0]==0):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[2][0], g=color[2][1], b=color[2][2]))
         
         if(str == "square"):
            if(math.sqrt((((coord[idx][1])**2) + ((coord[idx][0])**2))) == (math.sqrt(((L/2)**2)+((L/2)**2)))):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
               
            else:
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
               

         if(str == "circle"):        
            coord = self.des_formation_coords
            if((L/2) - coord[idx][1]>=0 and (L/2)-coord[idx][0]<=0):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[0][0], g=color[0][1], b=color[0][2]))
            
            if((L/2) - coord[idx][1]>=0 and (L/2)-coord[idx][0]>0):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[1][0], g=color[1][1], b=color[1][2]))
            
            if((L/2) - coord[idx][1]<0 and (L/2)-coord[idx][0]<=0):
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[2][0], g=color[2][1], b=color[2][2]))
            
            else:
               thrd = Thread(target=clover.set_effect, kwargs=dict(effect=effect, r=color[3][0], g=color[3][1], b=color[3][2]))
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
   
   #Transformations
   def transformFormation(self, sx, sy, sz, anglex, angley, anglez, tx, ty, tz):
      new_coord = transform.transformFormation(self.des_formation_coords, sx, sy, sz, anglex, angley, anglez, tx, ty, tz)
      self.des_formation_coords = new_coord
      self.des_formation_name = 'transform'
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def scaleFormation(self, sx, sy, sz):
      self.des_formation_coords = transform.scaleFormation(self.des_formation_coords, sx, sy, sz)
      self.des_formation_pose = np.array([self.des_formation_pose[0]*sx, self.des_formation_pose[1]*sy, self.des_formation_pose[2]*sz])
      self.des_formation_name = 'scale'
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def translateFormation(self, tx, ty, tz):
      self.des_formation_coords = transform.translateFormation(self.des_formation_coords, tx, ty, tz)
      self.des_formation_pose = np.array([self.des_formation_pose[0]+tx, self.des_formation_pose[1]+ty, self.des_formation_pose[2]+tz])
      self.des_formation_name = 'translate'
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   def rotateFormation(self, anglex, angley, anglez):
      # Get x, y, z of current formation
      tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
      # Translate back to the origin 
      origin_coords = transform.translateFormation(self.des_formation_coords, -tx, -ty, -tz)
      # Rotate formation on the origin
      origin_coords = transform.rotateFormation(origin_coords, anglex, angley, anglez)
      # Translate back to the current pose
      self.des_formation_coords = transform.translateFormation(origin_coords, tx, ty, tz)
      self.des_formation_name = 'rotate'
      self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
      self.op_num += 1

   #Leader operations
   def setLeader(self, id):
      assert type(id)==int, "Input 'id' must be an integer."
      self.leader_id = id
   
   def followLeader():
      pass

if __name__ == "__main__":

   
   print("Select the operation mode:")
   print("1 - Planning mode")
   print("2 - Simulation mode")
   print("3 - Navigation mode")
   selec_mode = input('\n')
   selec_amount = int(input(f"\nType the amount of clovers: "))
   swarm = Swarm(int(selec_amount))
   if (selec_mode == str('1')):
      # Starts the simulation just with the plots previews
      swarm.startPlanning()
   elif (selec_mode == str('2')):
      # Starts the Gazebo simulation and clovers ready to operate
      swarm.startSimulation(launch=True)
   elif (selec_mode == str('3')):
      pass
   else:
      logging.debug("There isn't this mode")
      rospy.loginfo("There isn't this mode")
      sys.exit()

   
   #Menu 
   def menu():
      print("Select")
      print("\n-----Basic operations-----")
      print("1 - Takeoff all")
      print("0 - Initial position")
      print("L - Land all")
      print("led - Set Led for all drones")
      print("\n-----Formations-----")
      print("2 - Line formation")
      print("3 - Triangle formation")
      print("4e - Empty square formation")
      print("4f - Full square formation")
      print("O - Circle formation")
      print("5 - Cube formation")
      print("6 - Sphere formation")
      print("7 - Pyramid formation")
      print("A - Alphabet formation")
      print("\n-----Transformations-----")
      print("TR - Rotate")
      print("TS - Scale")
      print("TT - Translate")
      print("\n-----Plot and Apply-----")
      print("AP - Apply formation")
      print("PLT - Plot preview")
      print("PLT3D - Plot 3D preview")
      print("FL - Formation list")
      print("\nE - Exit")

   N = swarm.num_of_clovers
   #init_form = swarm.setInitialPosition()
   
   while not rospy.is_shutdown():
      coord = swarm.des_formation_coords
      menu()
      key = input('\n')
      if (key == str('1')):
         swarm.takeOffAll()
         #rospy.sleep(2)

      elif (key == str('2')):
         if (N < 2):
            print("You need at least 2 clovers!\n")
         else:
            L = int(input("Insert the desired length: "))
            swarm.setFormation2D('line', N, L)
            #rospy.sleep(5)

      elif (key == str('3')):
         if (N < 3):
            print("You need at least 3 clovers!\n")
         else:
               L = int(input("Insert the desired side length: "))
               swarm.setFormation2D('triangle', N, L)
               #rospy.sleep(5)

      elif (key == str('4f') or key == str('4F')):
         if (N < 4):
            print("You need at least 4 clovers!\n")
         else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('full_square', N, L)
            #rospy.sleep(5)

      elif (key == str('4e') or key == str('4E')):
         if (N < 4):
            print("You need at least 4 clovers!\n")
         else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('empty_square', N, L)
            #rospy.sleep(5)

      elif (key == str('o') or key == str('O')):
         L = int(input("Insert the desired ratio: "))
         swarm.setFormation2D('circle', N, L)
         #rospy.sleep(2)

      elif (key == str('5')):
         if (N < 8):
               print("You need at least 8 clovers!\n")
         else:
               #type = input("Insert full or empty: ")
               L = int(input("Insert the desired side length: "))
               swarm.setFormation3D('cube', N, L)
               #rospy.sleep(5)

      elif (key == str('6')):
         L = int(input("Insert the desired ratio: "))
         swarm.setFormation3D('sphere', N, L)
         #rospy.sleep(5)

      elif (key == str('7')):
         if (N < 3):
               print("You need at least 3 clovers!\n")
         else:
               L = int(input("Insert the desired side length: "))
               swarm.setFormation3D('pyramid', N, L)
               rospy.sleep(5)

      elif (key == str('0')):
         swarm.returnToHome()
         rospy.sleep(2)
      
      elif  (key == str('a') or key == str('A')):
         user_input = input(f"Please, enter word or a letter: ")
         swarm.setAlphabet(user_input, N)

      elif (key == str('l') or key == str('L')):
         swarm.landAll()
         rospy.sleep(5)

      elif (key == str('ts') or key == str('TS')):
         sx = int(input("Insert the x scale: "))
         sy = int(input("Insert the y scale: "))
         sz = int(input("Insert the z scale: "))
         swarm.scaleFormation(sx, sy, sz)
      
      elif (key == str('tr') or key == str('TR')):
         anglex = float(input("Insert the x angle: "))*np.pi/180
         angley = float(input("Insert the y angle: "))*np.pi/180
         anglez = float(input("Insert the z angle: "))*np.pi/180
         swarm.rotateFormation(anglex, angley, anglez)

      elif (key == str('tt') or key == str('TT')):
         tx = int(input("Insert the x translation: "))
         ty = int(input("Insert the y translation: "))
         tz = int(input("Insert the z translation: "))
         swarm.translateFormation(tx, ty, tz)

      elif (key == str('ciranda')):
         ang=0
         while(ang < 4*np.pi):
            swarm.rotateFormation(0, 0, ang)
            rospy.sleep(2)
            swarm.applyFormation()
            rospy.sleep(3)

      elif (key == str('ap') or key == str('AP')):
         swarm.applyFormation()

      elif (key == str('plt') or key == str('PLT')):
         plot.create_swarm_preview(swarm, swarm.des_formation_coords, preview_type='2D')
      
      elif (key == str('plt3d') or key == str('PLT3D') or key == str('plt3D')):
         plot.create_swarm_preview(swarm, swarm.des_formation_coords, preview_type='3D')

      elif (key == str('fl') or key == str('FL')):
         print(swarm.formation_list)

      elif (key == str('led')):
         effect = str(input("input led effect: "))
         red = int(input("Insert the red color (0-255): "))
         green = int(input("Insert the green color (0-255): "))
         blue = int(input("Insert the blue color (0-255): "))
         swarm.ledAll(effect, red, green, blue)

      elif (key == str('led2')):
         effect = str(input("input led effect: "))
         red = int(input("Insert the red color (0-255): "))
         green = int(input("Insert the green color (0-255): "))
         blue = int(input("Insert the blue color (0-255): "))
         swarm.ledEven(effect, red, green, blue)

      elif (key == str('led3')):
         effect = str(input("input led effect: "))
         red = int(input("Insert the red color (0-255): "))
         green = int(input("Insert the green color (0-255): "))
         blue = int(input("Insert the blue color (0-255): "))
         swarm.ledOdd(effect, red, green, blue)
      
      elif (key == str('e') or key == str('E')):
         break