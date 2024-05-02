# Standard imports for having access to numpy, scipi, and matplotlib
import time
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

# Standard imports for the ROS Client Library (rcl)
import rclpy
from rclpy.node import Node

# Imports for each of the messages that will be used by the node.
from geometry_msgs.msg import Point

# Declare a new class. The "(Node)" indicates that this class derives from the ROS Node class
class PlottingSubscriber(Node):

    # Every class should have an __init__ function, which is the constructor.
    # This gets called each time a new variable of this type is created
    def __init__(self, fig, ax):
        # When your class is derived from another class, you should alway call the parent/super
        #  class's __init__ function also
        super().__init__('plotting_subscriber')

        # This creates a class variable that is a ROS subscriber. Because we derived this class
        # from the ROS Node class, it has access to class functions declared in the parent
        # class, like create_subscription.
        # The first parameter is the message type that this publisher will receive.
        # The second parameter is the name of the topic to listen on.
        # The third parameters is the function that will be called each time a message is received
        # The fourth parameter is the queue size, in case you are publishing very quickly and need
        #  them to be able to queue.
        self.subscription = self.create_subscription(
            Point,
            'coord',
            self.listener_callback,
            10)
        
        self.fig = fig
        self.ax = ax

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        t = time.time()
        x = 0.5 + np.cos(0.1*t)
        y = 0.5 + np.sin(0.1*t)

        self.ax.plot(x,y,'g*')    
    

    # This is the class function we are going to use to process incoming messages
    def listener_callback(self, msg:Point):

        # print('Message received')

        # We want to plot the point that is coming in, by just adding it to the current plot axis
        self.ax.plot(msg.x, msg.y, 'r*')

    def run_node_loop(self):

        # you need this line in there to let it do all the work of sending/receive messages and services
        rclpy.spin_once(self)

        # We put this here so that the actual drawing operation is taken care of on the main thread
        plt.draw()
        plt.pause(0.1)

        # NOTE: Could add other stuff that I want to run on the main thread.
        # ...

         
if __name__ == '__main__':

    # Creates a new Matplotlib plot
    fig, ax = plt.subplots()

    # Puts Matplotlib in interactive mode so that the plt.show() doesn't block execution
    plt.ion()
    plt.show()

    # We could plot outside the node's main loop if we need to
    plt.plot(0,0,'bo')
    plt.draw()
    plt.pause(1)


    # Same as before, we initialize the ROS2 client library
    rclpy.init()

    # Create the node. 
    # NOTE: We changed this node class so that we pass the figure and axis variables into the class to be stored
    minimal_subscriber = PlottingSubscriber(fig, ax)

    # NEW WAY OF RUNNING THE NODE'S run_node_loop FUNCTION MANUALLY
    while True:
        minimal_subscriber.run_node_loop()
        time.sleep(0.001)

    # OLD WAY OF LETTING THE NODE JUST RUN
    # rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()