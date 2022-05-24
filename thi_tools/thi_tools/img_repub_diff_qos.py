import rclpy
from rclpy import qos
from rclpy.node import Node

from sensor_msgs.msg import Image

class ImgRepubDiffQos(Node):

  def __init__(self):
    super().__init__('img_repub_diff_qos_node')
    self.pub_img = self.create_publisher(Image, 'image', qos.qos_profile_system_default)
    self.sub_img = self.create_subscription(Image, '/camera/rgb/image', self.sub_img_callback, qos.qos_profile_sensor_data)
    self.get_logger().info("init rdy")
    
  def sub_img_callback(self, msg):
    self.pub_img.publish(msg)
    self.get_logger().info("publish image")



def main(args=None):
  rclpy.init(args=args)

  img_repub_node = ImgRepubDiffQos()

  try:  # to prevent exception after Ctrl-C // this is still a bug???
    rclpy.spin(img_repub_node)
  except KeyboardInterrupt:
    pass

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  img_repub_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()