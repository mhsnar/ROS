# Standard imports for having access to numpy, scipi, and matplotlib
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

# Standard imports for the ROS Client Library (rcl)
import rclpy
from rclpy.node import Node

# Imports for each of the messages that will be used by the node.
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from turtlesim.srv import SetPen


# Declare a new class. The "(Node)" indicates that this class derives from the ROS Node class
class RobotController(Node):

    
    # Every class should have an __init__ function, which is the constructor.
    # This gets called each time a new variable of this type is created
    def __init__(self):
        # When your class is derived from another class, you should alway call the parent/super
        #  class's __init__ function also
        super().__init__('robot_controller')
        
        # This creates a class variable that is a ROS publisher. Because we derived this class
        # from the ROS Node class, it has access to class functions declared in the parent
        # class, like create_publisher.
        # The first parameter is the message type that this publisher will send.
        # The second parameter is the name of the topic.
        # The third parameters is the queue size, in case you are publishing very quickly and need
        #  them to be able to queue.
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)

        # Create both the service client and the request object for set_pen service
        self.setpen_service_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.setpen_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.setpen_request = SetPen.Request()
        self.pen_off = False

        # This code creates a timer that will be executed every 0.5 seconds.
        # It uses a class function from the Node class called create_timer.
        #  The first parameter is the period in seconds
        #  The second parameter is the function that will be called each time the timer triggers.
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.velocity_controller_callback)
        self.i = 1

        

    # This is the class function we are going to use to send a message out on the publisher
    def velocity_controller_callback(self):


        if self.i % 4 == 0:

            # Flip the pen_off variable between True/False
            self.pen_off = not self.pen_off

            # Fill in the service request value
            self.setpen_request.r = 255
            self.setpen_request.g = 0
            self.setpen_request.b = 0
            self.setpen_request.width = 3
            self.setpen_request.off = self.pen_off

            # Send the request to the service through the client
            self.future = self.setpen_service_client.call_async(self.setpen_request)

            # Wait for the service to complete
            # NOTE: In the video, I wasn't sure why this was blocking forever
            #       I found that this was because it was responding so fast that the service
            #       was already handled even before this function was called. So, the recommendation
            #       was to make sure you add a timeout so that you get past this if that timeout 
            #       duration elapses.
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.0)

            print('Done switching pen state')


        if self.i < 30:
            turn = -1
        else:
            turn = 1

        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.z = np.pi/8*turn
        msg.angular.y = 0.0
        msg.angular.x = 0.0
        
        # Create the message and publish it.
        self.publisher_.publish(msg)

        # Log a message to the ROS2 data logger as an informational message
        # You could just print() here, but the ROS2 logging is useful if you ever want
        # to record a bag file.
        # You can also use
        #   self.get_logger().warn()
        #   self.get_logger().error()
        # to categorize messages by severity.
        self.get_logger().info(f'Publishing: {msg}')

        # Increment the class variable 'i' so that the number increases with each message sent.
        self.i += 1




if __name__ == '__main__':
    rclpy.init()

    robot_controller = RobotController()

    rclpy.spin(robot_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()
