# Standard imports for having access to numpy, scipi, and matplotlib
import time
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from transform3d import Transform
from typing import List
from enum import Enum
from random import randint

# Standard imports for the ROS Client Library (rcl)
import rclpy
from rclpy.node import Node

# Imports for each of the messages that will be used by the node.
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

# Imports for saving the map
from nav2_msgs.srv import SaveMap

# Imports for navigating to different locations
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage

# Imports for processing the map and identifying waypoints
import cv2
from scipy.signal import convolve2d
from sklearn.cluster import KMeans


class ReplanStatus(Enum):
    INIT = 1
    PLAN_RUNNING = 2
    GOALREACHED = 3
    TIMEOUT = 4
    PLAN_FAILED = 5
    DONE = 6


class HouseExplorerNode(Node):

    first_map_received = False
    first_odometry_received = False

    replan_status = ReplanStatus.INIT
    
    PLAN_TIMEOUT = 20
    GOAL_DIST_THRESH = 0.5
    NUM_CLUSTERS = 4
    map = None

    rejected_points = []

    def __init__(self):
        super().__init__('house_explorer')
        
        # Create the action client for sending waypoints
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
        # Set up the client for the map_saver service
        self.map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')
        while not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('map_saver service not available, waiting again...')

        # Subscribe to the map and odometry
        self.odom_subscriber = self.create_subscription(Odometry,f'/odom',self.on_odom_received,1)
        self.map_subscriber = self.create_subscription(OccupancyGrid,f'/map',self.on_map_received,1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.main_thread_timer)

    def find_explore_goal_waypoint(self, do_random_cluster=False):
        pass
        print('Searching for waypoint')
        
        # Get the map image
        mapimg = self.map[0,:,:]
        
        # define the edge detection kernel
        kernel = np.array([[-1, -1, -1],
                           [-1,  8, -1],
                           [-1, -1, -1]])

        # Perform the edge detection
        edges = convolve2d(mapimg, kernel, mode='same', boundary='symm')
        edges = np.abs(edges)
        edges[edges>50] = 0
        edges = 255.0 * edges/np.max(edges)

        rows, cols = np.where(edges > 128)

        x = self.map[1,rows,cols]
        y = self.map[2,rows,cols]
        points = np.column_stack((x, y))

        # If too few points of bordering empty and unknown were found, then we are done with the search
        if points.shape[0] < 20:
            return np.zeros((2,)), False

        # Create a KMeans instance with k clusters
        kmeans = KMeans(n_clusters=self.NUM_CLUSTERS)

        # Fit the model to your data (compute k-means clustering)
        kmeans.fit(points)

        # Predict the closest cluster each sample in points belongs to
        labels = kmeans.predict(points)

        # Centroids of the clusters
        centroids = kmeans.cluster_centers_
        # print(f'PREFILTER Cluster centroids: {centroids}')
        
        # Filter out points that we have previously rejected as unreachable because we came too close and it tried to replan there
        valid_idx = []
        for I in range(centroids.shape[0]):
            too_close = False
            for J in range(len(self.rejected_points)):
                centroid = centroids[I,:]
                rejected_point = self.rejected_points[J]
                dist = np.linalg.norm(centroid-rejected_point)
                if dist < 0.2:
                    too_close = True
            if too_close == False:
                valid_idx.append(I)
        centroids = centroids[valid_idx,:]
        # print(f'POSTFILTER Cluster centroids: {centroids}')

        if do_random_cluster:
            idx = randint(0,centroids.shape[0]-1)
        else:
            idx = 0
            mindist = 1000
            throwaway_dist = 1.0

            for I in range(centroids.shape[0]):
                dist = np.linalg.norm(centroids[I,:] - self.pose.p[0:2])
                if dist < mindist and dist > throwaway_dist:
                    mindist = dist
                    idx = I
                
                if dist < throwaway_dist:
                    self.rejected_points.append(centroids[I,:])

            
        
        return centroids[idx,:], True
        
        


    def main_thread_timer(self):
        
        # Make sure at least one map and odometry have been received before trying to plan
        if self.first_map_received is False or self.first_odometry_received is False:
            print('Waiting for first map and odometry before planning')
            return
        
        # Here I need to put some intelligence
        # If we are ready for a re-plan, then take the latest map, find the next location, then send a goal
        if self.replan_status == ReplanStatus.DONE:
            pass
        elif self.replan_status != ReplanStatus.PLAN_RUNNING:
            pass
            # Include options for handling re-plans from different causes differently (e.g. for a failed plan, should pick a different point)
            if self.replan_status == ReplanStatus.INIT or self.replan_status == ReplanStatus.TIMEOUT:
                print('Making new plan for normal reasons')
                self.goal, valid = self.find_explore_goal_waypoint()
                print(f'NEW GOAL: {self.goal}')
                self.replan_status = ReplanStatus.PLAN_RUNNING
            elif self.replan_status == ReplanStatus.GOALREACHED:
                print('Making new plan because the goal was reached')
                self.goal, valid = self.find_explore_goal_waypoint()
                print(f'NEW GOAL: {self.goal}')
                self.replan_status = ReplanStatus.PLAN_RUNNING
            elif self.replan_status == ReplanStatus.PLAN_FAILED:
                print('Making a new plan because the old plan failed')
                self.goal, valid = self.find_explore_goal_waypoint(do_random_cluster=True)
                print(f'NEW GOAL: {self.goal}')
                self.replan_status = ReplanStatus.PLAN_RUNNING
            
            if valid:
                self.send_goal(self.goal[0],self.goal[1])
            else:   
                print('********* SEARCHING DONE BECAUSE NO POINTS LEFT TO FIND *********')             
                self.replan_status = ReplanStatus.DONE
            self.last_plan_time = time.time()
            self.last_plan_pose = self.pose

        # If not ready for a re-plan, monitor conditions to see if a re-plan should be done. 
        else:            
            # The following things could trigger a re-plan:
            #  1. Close enough to the goal
            #  2. Time elapsed (and maybe without enough progress)
            #  3. Goal feedback indicates a failed plan (probably need to also cause this to trigger an alternative planning approach)
            dist_to_goal = np.linalg.norm(self.pose.p[0:2] - self.goal)
            
            if (time.time() - self.last_plan_time) > self.PLAN_TIMEOUT:
                print('Plan timeout occurred')
                self.replan_status = ReplanStatus.TIMEOUT
            elif dist_to_goal < self.GOAL_DIST_THRESH:
                print('Plan because goal is reached')
                self.replan_status = ReplanStatus.GOALREACHED
            elif (time.time() - self.last_plan_time) > 3.0*self.PLAN_TIMEOUT/5.0 and np.linalg.norm(self.pose.p - self.last_plan_pose.p) < 0.25:
                print('Plan because we did not move enough since the last plan')
                self.replan_status = ReplanStatus.PLAN_FAILED
            # TODO: Check to see if our position isn't changing enough

    ## Functions related to the odometry and map subscribers
    def on_odom_received(self, msg:Odometry):
        #print('Odometry received')

        # Extract the current position and orientation so that we can use it for determining where to go next
        p = [msg.pose.pose.position.x, msg.pose.pose.position.y, 0]
        q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.pose = Transform(p=p, quat=q)
        
        # At the end, indicate the first odometry has been received and stores so that the planner can start functioning
        self.first_odometry_received = True

    def on_map_received(self, msg:OccupancyGrid):
        #print('Map received')

        # Save the last map for comparison with the new map
        if self.map is not None:
            last_map = self.map.copy()
        
        # Extract the current map so that we can use it to calculate where to explore next
        mapdata=np.reshape(msg.data,(msg.info.height,msg.info.width))

        # TODO: Store this as a 3xNXM map where each layer is [value, x, y]. This should make it much easier to do the computations
        xrange = msg.info.origin.position.x + np.arange(0,msg.info.width*msg.info.resolution,msg.info.resolution)
        yrange = msg.info.origin.position.y + np.arange(0,msg.info.height*msg.info.resolution,msg.info.resolution)
        xv,yv = np.meshgrid(xrange,yrange, indexing='xy')
        newmap = np.stack((mapdata,xv,yv), axis=0)

        # If the map has changed, save the map to file so that we always have the latest version
        # TODO: do the comparisons with the old map: (1) matrix sized changed or (2) values of map changed
        if self.map is None or newmap.shape != last_map.shape or not np.array_equal(newmap,last_map):
            print('Saving map because it has changed')
            self.map = newmap
            self.save_map()

        # At the end, indicate the first map has been received and stores so that the planner can start functioning
        self.first_map_received = True

    ## Functions related to the map_saver
    def save_map(self):
        request = SaveMap.Request()
        # Set the request parameters
        request.map_topic = '/map'
        request.map_url = '/home/me485/pymap'
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self.future = self.map_saver_client.call_async(request)

    ## Functions related to the robot navigation
    def send_goal(self, x, y, theta=0):
        print(f'Sending new waypoint: ({x},{y},{theta})')

        # Creat the goal pose object 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"

        # Fill in the desired (x,y) position
        goal_pose.pose.position.x = x  # Specify x position
        goal_pose.pose.position.y = y  # Specify y position

        # Get the quaternion from the desired rotation about z and fill in the desired rotation
        rotation = Transform(rotvec=[0,0,theta])
        goal_pose.pose.orientation.x = rotation.quat[0]
        goal_pose.pose.orientation.y = rotation.quat[1]
        goal_pose.pose.orientation.z = rotation.quat[2]
        goal_pose.pose.orientation.w = rotation.quat[3]

        # Create the goal message and send it to the navigation action server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # This is the callback for the feedback while the goal is being processed
    def goal_feedback_callback(self, feedback_msg:NavigateToPose_FeedbackMessage):
        # print(f'Feedback Message: {feedback_msg}')
        pass
        # Note: if distance_remaining is 0.0, then the plan failed
        # if feedback_msg.feedback.distance_remaining == 0.0:
        #     print('Found an infeasible plan....Switching to FAILED_PLAN ')
        #     self.replan_status = ReplanStatus.PLAN_FAILED


    def goal_response_callback(self, future):
        
        # Note: There are some problems with the ROS2 navigation, where even an unattainable goal will be accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Set up a callback to know when the goal is reached...I probably won't really use this and be checking the distance myself
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

        # if result.status == NavigateToPose.Result.SUCCESS:
        #     self.get_logger().info('Goal reached successfully.')
        # else:
        #     self.get_logger().info('Failed to reach the goal. Status: {0}'.format(result.status))



if __name__ == '__main__':
    rclpy.init(args=None)

    house_explorer = HouseExplorerNode()
    rclpy.spin(house_explorer)
    rclpy.shutdown()



