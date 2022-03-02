import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap


class MapRepubNode(Node):

  def __init__(self):
    super().__init__('map_repub_node')
    self.publisher_ = self.create_publisher(OccupancyGrid, 'map/repub', 10)
    timer_period = 5  # seconds
    self.timer_pub_map = self.create_timer(timer_period, self.timer_callback_pub_map)
    self.timer_req_map = self.create_timer(1, self.timer_callback_request_map)
    self.map = OccupancyGrid()
    self.got_map = False
    self.sent_srv = False
    #service
    self.get_map_client = self.create_client(GetMap, '/map_server/map')
    while not self.get_map_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service unavailable, retry...')
    self.req_map = GetMap.Request()

  def timer_callback_pub_map(self):
    # self.get_logger().info('pub_map cb')
    if self.got_map:
      self.publisher_.publish(self.map)
      # self.get_logger().info("publish map")
    pass
  
  def timer_callback_request_map(self):
    # self.get_logger().info('req_map cb')
    
    if not self.sent_srv:
      self.get_map_future : GetMap.Response = self.get_map_client.call_async(self.req_map)
      self.sent_srv = True
    elif self.get_map_future.done():
      
      try:
        self.get_map_rsp = self.get_map_future.result()
      except Exception as e:
        self.get_logger().info('Service call failed %r' % (e,))
      else:
        #log info about map
        self.get_logger().info('++++++++++++++++++++++++++++++++++++++++++')
        self.get_logger().info('++  Got Map via Service:  ++++++++++++++++')
        self.get_logger().info('resolution: %f' % self.get_map_rsp.map.info.resolution)
        self.get_logger().info('width     : %d' % self.get_map_rsp.map.info.width)
        self.get_logger().info('height    : %d' % self.get_map_rsp.map.info.height)
        # self.get_logger().info('origin    : ', self.get_map_rsp.map.info.origin)
        self.get_logger().info('++++++++++++++++++++++++++++++++++++++++++')
        self.map = self.get_map_rsp.map
        self.got_map = True
        self.sent_srv = False
        self.timer_req_map.cancel()




def main(args=None):
  rclpy.init(args=args)

  map_repub_node = MapRepubNode()

  rclpy.spin(map_repub_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  map_repub_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()