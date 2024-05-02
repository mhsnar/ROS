# Standard imports for having access to numpy, scipi, and matplotlib
import time
import os
from pathlib import Path
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from transform3d import Transform
from typing import List
from enum import Enum
from random import randint
import yaml

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


class FinderStatus(Enum):
    INIT = 1
    ANALYZE_MAP = 2
    PLANNING = 3
    DRIVE = 4
    DONE = 5


class TrashcanFinder(Node):

    first_map_received = False
    first_odometry_received = False

    finder_state = FinderStatus.INIT
    PLAN_TIMEOUT = 20
    GOAL_DIST_THRESH = 1.0
    
    map = None

    rejected_points = []

    waypoints = []

    def __init__(self):
        super().__init__('trashcan_finder')
        
        # Create the action client for sending waypoints
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
        # Subscribe to the map and odometry
        self.odom_subscriber = self.create_subscription(Odometry,f'/odom',self.on_odom_received,1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.main_thread_timer)

    def read_map_from_file(self,fn='~/map.yaml'):
        pass
        # TODO: Read in the map and convert to the 3xHxW format from Part 1
        with open(fn, 'r') as file:
            # Use safe_load to read the YAML content
            yaml_content = yaml.safe_load(file)
            image_fn = yaml_content['image']
            res = yaml_content['resolution']
            origin = yaml_content['origin']

            path = Path(fn).resolve()
            directory_path = path.parent
            map_img_fn = str(directory_path) + '/' + image_fn

            mapdata = cv2.imread(map_img_fn, cv2.IMREAD_GRAYSCALE)
            mapdata = np.flipud(mapdata) # imread and images normally have x=0,y=0 in upper left

            xrange = origin[0] + np.arange(0,mapdata.shape[1]*res, res)
            yrange = origin[1] + np.arange(0,mapdata.shape[0]*res, res)
            xv,yv = np.meshgrid(xrange,yrange, indexing='xy')
            self.map = np.stack((mapdata,xv,yv), axis=0)

        # TODO: Search for the garbage cans and store candidate locations in waypoint list
        threshimg = mapdata < 225 # Find all the unknown spots and walls
        filled_circle_image = 255*threshimg.astype(np.uint8)
        circles = cv2.HoughCircles(filled_circle_image,cv2.HOUGH_GRADIENT,dp=1, minDist=50,
                            param1=50,param2=7.9,minRadius=6,maxRadius=13)

        for row in np.squeeze(circles):
            x_map = self.map[1,round(row[1]),round(row[0])]
            y_map = self.map[2,round(row[1]),round(row[0])]
            self.waypoints.append(np.array([x_map,y_map]))

        print(f'Waypoints from map analysis: {self.waypoints}')
        

    def plan_path_to_waypoint(self, do_random_waypoint=False):
        if len(self.waypoints) == 0:
            return np.zeros((2,)), False

        if do_random_waypoint:
            idx = randint(0,len(self.waypoints)-1)
        else:
            idx = 0
            mindist = 1000
            throwaway_dist = 1.0

            for I in range(len(self.waypoints)):
                        
                dist = np.linalg.norm(self.waypoints[I] - self.pose.p[0:2])
                if dist < mindist:
                    mindist = dist
                    idx = I
        
        return self.waypoints[idx], True

    def remove_waypoint(self, point):
        filtered_list = [arr for arr in self.waypoints if not np.array_equal(arr, point)]
        self.waypoints = filtered_list

    def main_thread_timer(self):
        
        # Make sure at least one map and odometry have been received before trying to plan
        if self.first_odometry_received is False:
            print('Waiting for first map and odometry before planning')
            return
        
        # Here I need to put some intelligence
        if self.finder_state == FinderStatus.DONE:
            pass
        elif self.finder_state != FinderStatus.DRIVE:
            if self.finder_state == FinderStatus.INIT:
                print('Startup robot')
                self.finder_state = FinderStatus.ANALYZE_MAP
            elif self.finder_state == FinderStatus.ANALYZE_MAP:
                print('Analyzing Map')
                self.read_map_from_file('/home/me485/pymap.yaml')
                self.finder_state = FinderStatus.PLANNING
            elif self.finder_state == FinderStatus.PLANNING:
                self.goal, valid = self.plan_path_to_waypoint()
                if valid:
                    print(f'NEW GOAL: {self.goal}')
                    self.send_goal(self.goal[0],self.goal[1])
                    self.finder_state = FinderStatus.DRIVE
                self.last_plan_time = time.time()
                self.last_plan_pose = self.pose


        elif self.finder_state == FinderStatus.DRIVE:
            
            dist_to_goal = np.linalg.norm(self.pose.p[0:2] - self.goal)

            if (time.time() - self.last_plan_time) > self.PLAN_TIMEOUT:
                print('Plan timeout occurred')
                self.finder_state = FinderStatus.PLANNING

            elif dist_to_goal < self.GOAL_DIST_THRESH:
                print('Plan because goal is reached')
                
                # Remove the goal from the list of waypoints and trigger a replan
                self.remove_waypoint(self.goal)

                # If not waypoint are left, we are done
                # Else trigger a new replan            
                if len(self.waypoints) == 0:
                    print('Task completed because I have visited all waypoints')
                    self.finder_state = FinderStatus.DONE
                else:
                    self.finder_state = FinderStatus.PLANNING

            

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

    ## Functions related to the robot navigation
    def send_goal(self, x, y, theta=0):
        print(f'Sending new waypoint: ({x},{y},{theta})')

        # Creat the goal pose object 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"

        # Fill in the desired (x,y) position
        goal_pose.pose.position.x = float(x)  # Specify x position
        goal_pose.pose.position.y = float(y)  # Specify y position

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

    trashcan_finder = TrashcanFinder()
    rclpy.spin(trashcan_finder)
    rclpy.shutdown()



