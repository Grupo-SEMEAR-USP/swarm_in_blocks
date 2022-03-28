#!/usr/bin/python3

import rospy
import tf2_ros

from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from swarm_checker.msg import SwarmState

import numpy as np
from threading import Thread
import traceback
import logging

class SingleClover:
    # Create and call all servicers, subscribers and clover topics
    def __init__(self, name, clover_id):
        self.name = name
        self.id = clover_id
        self.init_coord = np.array([])
        self.pose = np.array([])

        # Configure clover services and topics
        self.configure()

    def __localPositionCallback(self, local_position):
        
        if self.init_coord.size != 0:
            x = local_position.pose.position.x + self.init_coord[0]
            y = local_position.pose.position.y + self.init_coord[1]
            z = local_position.pose.position.z + self.init_coord[2]
            self.pose = np.array([x, y, z])

    def configure(self):

        logging.debug("Waiting clover services...")
        rospy.loginfo("Waiting clover services...")

        rospy.wait_for_message(f"{self.name}/mavros/local_position/pose", PoseStamped, timeout=1)
        rospy.Subscriber(f"{self.name}/mavros/local_position/pose", PoseStamped, self.__localPositionCallback)

        rospy.wait_for_service(f"{self.name}/get_telemetry", timeout=1)
        self.get_telemetry = rospy.ServiceProxy(
            f"{self.name}/get_telemetry", srv.GetTelemetry)

        rospy.wait_for_service(f"{self.name}/navigate", timeout=1)
        self.navigate = rospy.ServiceProxy(
            f"{self.name}/navigate", srv.Navigate)

        rospy.wait_for_service(f"{self.name}/navigate_global", timeout=1)
        self.navigate_global = rospy.ServiceProxy(
            f"{self.name}/navigate_global", srv.NavigateGlobal)

        rospy.wait_for_service(f"{self.name}/set_position", timeout=1)
        self.set_position = rospy.ServiceProxy(
            f"{self.name}/set_position", srv.SetPosition)

        rospy.wait_for_service(f"{self.name}/set_velocity", timeout=1)
        self.set_velocity = rospy.ServiceProxy(
            f"{self.name}/set_velocity", srv.SetVelocity)

        rospy.wait_for_service(f"{self.name}/set_attitude", timeout=1)
        self.set_attitude = rospy.ServiceProxy(
            f"{self.name}/set_attitude", srv.SetAttitude)

        rospy.wait_for_service(f"{self.name}/set_rates", timeout=1)
        self.set_rates = rospy.ServiceProxy(
            f"{self.name}/set_rates", srv.SetRates)

        rospy.wait_for_service(f"{self.name}/land", timeout=1)
        self.land = rospy.ServiceProxy(f"{self.name}/land", Trigger)

        rospy.wait_for_service(f"{self.name}/led/set_effect", timeout=1)
        self.set_effect = rospy.ServiceProxy(
            f"{self.name}/led/set_effect", srv.SetLEDEffect)

    def navigateWait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):

        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed,
                      frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(
                frame_id='navigate_target' + str(self.id))
            if np.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

class SwarmInternalCollisionAvoidance():
    def __init__(self):
        # minimum secure distance (meters)
        self.dist_threshold = 1.5

        # ids handling for collision avoidance
        self.handling_ids = []

        self.all_of_clovers = 0
        self.connected_clovers = 0
        self.armed_clovers = 0
        self.offboard_mode_clovers = 0
        # Ids
        self.all_clovers_ids = []
        self.connected_ids = []

        # Initial formation
        self.init_formation_coords = []
        
        # list of clover objects
        self.swarm = []

        # init node
        rospy.init_node("collision_avoidance")
        # subscribe swarm checker to get clovers on network
        self.__subscribeSwarmChecker()
        # listen transformation to analyse clovers targets
        self.__initTF2Listener()
        # get initial formation
        self.__getInitialFormation()
        # create a list of clover objects
        self.__createCloversObjects()

    def __subscribeSwarmChecker(self):
        rospy.loginfo("Trying to connect to SwarmChecker node...")
        try:
            first_msg = rospy.wait_for_message("swarm_checker/state", SwarmState, timeout=10)
            self.swarm_checker_sub = rospy.Subscriber("swarm_checker/state", SwarmState, self.__swarmStateCallback)
        except Exception as e:
            rospy.logerr(e.__str__() + "\n Have you launched the simulation or the swarm_checker node?")
            exit()
            
        rospy.loginfo("SwarmApi connected to SwarmChecker.")

        # Configure first time
        self.swarm_state = first_msg
        self.all_of_clovers = self.swarm_state.all_clovers
        self.connected_clovers =self.swarm_state.connected_clovers
        self.armed_clovers = self.swarm_state.armed_clovers

        self.all_clovers_ids = self.swarm_state.all_clovers_ids
        self.connected_ids = self.swarm_state.connected_ids
    
    def __initTF2Listener(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    
    def __swarmStateCallback(self, msg):
        self.swarm_state = msg
        self.all_of_clovers = self.swarm_state.all_clovers
        self.connected_clovers =self.swarm_state.connected_clovers
        self.armed_clovers = self.swarm_state.armed_clovers

        self.all_clovers_ids = self.swarm_state.all_clovers_ids
        self.connected_ids = self.swarm_state.connected_ids
    
    def __getInitialFormation(self):
        coords = np.zeros((self.all_of_clovers, 4))
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

        self.init_formation_coords = coords

        # Log the result
        rospy.loginfo(f"{self.all_of_clovers} retrieved their initial pose.")
        for clover_id in initial_pose_failed:
            rospy.loginfo(f"Failed to get initial pose from clover {clover_id}")
        rospy.loginfo(f"Sucessfully got initial pose from {self.all_of_clovers-len(initial_pose_failed)} clovers, failed with {len(initial_pose_failed)} clovers.")

    # Create clover objects and append to clover object list
    def __createCloversObjects(self):
        for idx, clover_id in enumerate(self.connected_ids):
            clover_object = SingleClover(f"clover{clover_id}", clover_id)
            clover_object.init_coord = self.init_formation_coords[idx]
            self.swarm.append(clover_object)
    
    def __getDistance(self, pose_i, pose_j):
        if len(pose_i) != 0 and len(pose_j) != 0:
            dist = np.sqrt(np.sum(np.power(pose_i-pose_j,2)))
        else:
            dist = np.nan
        return dist
    
    def __checkParalellTrajectory(self, clover_id_i, clover_id_j):

        # clover_i = self.swarm[clover_id_i]
        # clover_j = self.swarm[clover_id_j]

        # telem_target_i = clover_i.get_telemetry(frame_id=f'navigate_target{clover_id_i}')
        # telem_target_j = clover_j.get_telemetry(frame_id=f'navigate_target{clover_id_j}')

        # traj_vec_i = [telem_target_i.x, telem_target_i.y, telem_target_i.z]
        # traj_vec_j = [telem_target_j.x, telem_target_j.y, telem_target_j.z]
        
        trans_target_i = self.tfBuffer.lookup_transform(f'base_link{clover_id_i}', f'navigate_target{clover_id_i}', rospy.Time())
        trans_target_j = self.tfBuffer.lookup_transform(f'base_link{clover_id_j}', f'navigate_target{clover_id_j}', rospy.Time())
        
        traj_vec_i = [trans_target_i.transform.translation.x, trans_target_i.transform.translation.y, trans_target_i.transform.translation.z]
        traj_vec_j = [trans_target_j.transform.translation.x, trans_target_j.transform.translation.y, trans_target_j.transform.translation.z]

        angle = np.arccos(np.clip(np.dot(traj_vec_i, traj_vec_j)/np.linalg.norm(traj_vec_i)/np.linalg.norm(traj_vec_j), -1.0, 1.0))
        
        if angle*np.pi <= 20 or angle >= 160:
            is_parallel = True
        else:
            is_parallel = False
        
        return is_parallel

    def __analyseDistance(self, poses, threshold):
        # distance matrix n,n 
        distance_mtx = np.empty((poses.shape[0],poses.shape[0]))
        distance_mtx[:] = np.nan
        near_ids = []

        # fill distance mtx and analyse distance if is lower than the thresh
        for i, pose_i in enumerate(poses):
            for j, pose_j in enumerate(poses):
                # Only elements above principal axis of the matrix must be completed
                # to prevent duplicated distances
                if i == j:
                    distance_mtx[i][j] = 0.0
                elif np.isnan(distance_mtx[j][i]):
                    dist = self.__getDistance(pose_i, pose_j)
                    distance_mtx[i][j] = dist

                    if dist <= threshold:
                        if ((j,i, True) in near_ids) or ((j,i,False) in near_ids):
                            continue
                        else:
                            is_parallel = self.__checkParalellTrajectory(i, j)
                            near_ids.append((i,j, is_parallel))
        return distance_mtx, near_ids
    
    def __stopClover(self, clover_id):
        # trans_base_link = self.tfBuffer.lookup_transform('map', f'base_link{clover_id}', rospy.Time())
        # x = trans_base_link.transform.translation.x
        # y = trans_base_link.transform.translation.y
        # z = trans_base_link.transform.translation.z
        clover = self.swarm[clover_id]
        x = clover.pose[0]
        y = clover.pose[1]
        z = clover.pose[2]
        rospy.loginfo(f"Semaphore red for {clover_id}. Stopping clover{clover_id} in target ({x},{y},{z}, frame_id='map') until another clover pass.")
        clover.navigateWait(x=x, y=y, z=z, yaw=float('nan'), speed=2.0, frame_id='map', auto_arm=False, tolerance=0.2)

    def __goToLastTarget(self, clover_id, traj_vec):
        x = traj_vec[0]
        y = traj_vec[1]
        z = traj_vec[2]
        rospy.loginfo(f"Semaphore green for {clover_id}. Navigating to original navigate target ({x},{y},{z}, frame_id='map')")

        # Dont use navigateWait because another clover has to see it position while this clover is navigating
        self.swarm[clover_id].navigate(x=x, y=y, z=z, yaw=float('nan'), frame_id='map')

    def __handleNonParallelClovers(self,  clover_id_i, clover_id_j):
        trans_target_i = self.tfBuffer.lookup_transform('map', f'navigate_target{clover_id_i}', rospy.Time())
        trans_target_j = self.tfBuffer.lookup_transform('map', f'navigate_target{clover_id_j}', rospy.Time())

        # Using threads to guarantee that both clovers stoped on the secure place to continue navigation
        stop_thrd_i = Thread(target=self.__stopClover, args=(clover_id_i,))
        stop_thrd_j = Thread(target=self.__stopClover, args=(clover_id_j,))

        stop_thrd_i.start()
        stop_thrd_j.start()

        stop_thrd_i.join()
        stop_thrd_j.join()
        
        traj_vec_i = [trans_target_i.transform.translation.x, trans_target_i.transform.translation.y, trans_target_i.transform.translation.z]
        traj_vec_j = [trans_target_j.transform.translation.x, trans_target_j.transform.translation.y, trans_target_j.transform.translation.z]

        if clover_id_i > clover_id_j:
            stopped_id = clover_id_i
            self.__goToLastTarget(clover_id_j, traj_vec_j)
        else:
            stopped_id = clover_id_j
            self.__goToLastTarget(clover_id_i, traj_vec_i)
        
        while not rospy.is_shutdown():
            clover_i = self.swarm[clover_id_i]
            clover_j = self.swarm[clover_id_j]

            pose_i = clover_i.pose
            pose_j = clover_j.pose

            dist = self.__getDistance(pose_i, pose_j)

            trans_target_i = self.tfBuffer.lookup_transform(f'base_link{clover_id_i}', f'navigate_target{clover_id_i}', rospy.Time())
            trans_target_j = self.tfBuffer.lookup_transform(f'base_link{clover_id_j}', f'navigate_target{clover_id_j}', rospy.Time())

            # if distance is more than 1 times the clover distance thresh
            # go ahead and continue the last trajectory
            if dist > 1*self.dist_threshold:
                break
        
        # navigate to last target
        if stopped_id == clover_id_i:
            self.__goToLastTarget(clover_id_i, traj_vec_i)
        else:
            self.__goToLastTarget(clover_id_j, traj_vec_j)

    def __handleParallelClovers(self, clover_id_i, clover_id_j):
        pass 


    def __handleTwoNearClovers(self, clover_id_i, clover_id_j, is_parallel):
        
        rospy.loginfo(f"Handling clover{clover_id_i} and clover{clover_id_j} due to possible collision with parallel trajectories.")
        self.handling_ids.append((clover_id_i, clover_id_j))
        if is_parallel:
            self.__handleNonParallelClovers(clover_id_i, clover_id_j)
        else:
            self.__handleNonParallelClovers(clover_id_i, clover_id_j)
        rospy.loginfo(f"Finished handling clover{clover_id_i} and clover{clover_id_j}. Minimum distance achievied: X")
        self.handling_ids.remove((clover_id_i, clover_id_j))

    def avoidCollision(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            poses = []
            for clover in self.swarm:
                poses.append(clover.pose)
            poses = np.array(poses)

            distance_mtx, near_ids = self.__analyseDistance(poses, self.dist_threshold)
            
            threads = []
            for clover_id_i, clover_id_j, is_parallel in near_ids:
                
                if ((clover_id_i, clover_id_j) in self.handling_ids) or ((clover_id_j, clover_id_i) in self.handling_ids):
                    # already handling these clovers
                    continue
                else:
                    thrd = Thread(target=self.__handleTwoNearClovers, args=(clover_id_i, clover_id_j, is_parallel))
                    threads.append(thrd)
                    thrd.start()
            rate.sleep()
        
if __name__ == '__main__':
    avoid = SwarmInternalCollisionAvoidance()
    avoid.avoidCollision()
    rospy.spin()