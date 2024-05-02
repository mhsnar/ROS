import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        self.client = self.create_client(SaveMap, '/map_saver/save_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_request(self):
        request = SaveMap.Request()
        # Set the request parameters
        request.map_topic = '/map'
        request.map_url = '/home/me485/pymap'
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65


        self.future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    map_saver_client = MapSaverClient()
    map_saver_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(map_saver_client)
        if map_saver_client.future.done():
            try:
                response = map_saver_client.future.result()
            except Exception as e:
                map_saver_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                #map_saver_client.get_logger().info(
                #    'Map saved: %r' % (response.success,))
                print(f'Response: {response}')
            break

    map_saver_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
