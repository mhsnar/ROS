import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

def main(args=None):
    rclpy.init(args=args)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = -0.5  # Specify x position
    goal_pose.pose.position.y = 0.5  # Specify y position
    goal_pose.pose.orientation.z = 0.0  # Specify orientation if needed
    goal_pose.pose.orientation.w = 1.0  # Specify orientation if needed

    nav2_goal_client = Nav2GoalClient()
    nav2_goal_client.send_goal(goal_pose)
    rclpy.spin(nav2_goal_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
