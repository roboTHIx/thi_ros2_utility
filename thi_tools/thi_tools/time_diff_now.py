import sys
import copy

import rclpy
from rclpy.node import Node

# import dataclasses

from tf2_msgs.msg import TFMessage
# from std_msgs.msg import String
# from geometry_msgs.msg import PointStamped
from ros2topic.api import get_msg_class


# class TfData(object):
#   def _init_(self):
#     self.
#     pass


class TimeDiffHandleTf(object):
  def __init__(self, parent : Node, topic,  max_buffer_size = 10):
    self.parent = parent
    self.buffer_size = max_buffer_size
    self.data =  {} #create dict
    
    self.got_data = False
    self.topic = copy.deepcopy(topic)

    #create subscirber
    msg_type = get_msg_class(parent, topic, include_hidden_topics=True)
    self.sub = parent.create_subscription(msg_type, topic, self.msg_callback, 10)
  
  def clear(self):
    self.parent.destroy_subscription(self.sub)

  def msg_callback(self, msg : TFMessage):
    #todo handle each frame pair extra
    for i in range(len(msg.transforms)):
      self.got_data = True
      now = self.parent.get_clock().now()
      tmp_diff_sec =  now.to_msg().sec - msg.transforms[i].header.stamp.sec
      tmp_diff_nsec = now.to_msg().nanosec - msg.transforms[i].header.stamp.nanosec
      # print(msg.transforms[i].header.frame_id, '------------')
      # print(msg.transforms[i].header.stamp)
      # print(now.to_msg())

      diff_s = tmp_diff_sec + tmp_diff_nsec/1000000000
      # print(len(self.data))
      # print(diff_s)

      key = msg.transforms[i].header.frame_id + ' -> ' + msg.transforms[i].child_frame_id
      # print("key: ", key)

      #push to list
      if key in self.data:
        self.data[key].append(diff_s)
      else:
        self.data[key] = [ diff_s ]

      #remove elemten is data size is grater / eq than buffer_size
      if len(self.data[key]) > self.buffer_size:
        del self.data[key][0]
  

  # def getTimeDiff(self):
  #   #compute average value
  #   avg = 0
  #   for i in range(len(self.data)):
  #     avg += self.data[i]
  #   avg = avg / len(self.data)
  #   return avg

  def hasData(self) -> bool:
    return self.got_data
  
  def getTopic(self):
    return self.topic

  def printTfMessage(self):
    for key, value in self.data.items():
      #compute average
      avg = 0
      for i in range(len(value)):
        # print('hans ', i, 'data: ', value[i])
        avg += value[i]
      avg = avg / len(value)
      print(key, ' : ', avg, ' s')



class TimeDiffHandle(object):
  def __init__(self, parent : Node, topic,  max_buffer_size = 10):
    self.parent = parent
    self.buffer_size = max_buffer_size;
    self.data =  [] #create list
    self.valid = True
    self.got_data = False
    self.topic = copy.deepcopy(topic)

    #create subscirber
    msg_type = get_msg_class(parent, topic, include_hidden_topics=True)
    self.sub = parent.create_subscription(msg_type, topic, self.msg_callback, 10)
  
  def clear(self):
    self.parent.destroy_subscription(self.sub)

  def msg_callback(self, msg):
    #chekc if msg has header for time stamp calc
    if not hasattr(msg, 'header'):
      self.valid = False
      # print("++++++++topic: ", self.topic, "is not valid!!!")
      return

    self.got_data = True
    now = self.parent.get_clock().now()
    # print('now: ', now.to_msg())
    # print('msg: ', msg.header.stamp)
    tmp_diff_sec =  now.to_msg().sec - msg.header.stamp.sec
    tmp_diff_nsec = now.to_msg().nanosec - msg.header.stamp.nanosec

    diff_s = tmp_diff_sec + tmp_diff_nsec/1000000000
    # print(len(self.data))
    #remove elemten is data size is grater / eq than buffer_size
    if len(self.data) >= self.buffer_size:
      del self.data[0]

    #push to list
    self.data.append(diff_s)

  def getTimeDiff(self):
    #compute average value
    avg = 0
    for i in range(len(self.data)):
      avg += self.data[i]
    avg = avg / len(self.data)
    return avg

  def isValid(self) -> bool:
    return self.valid

  def hasData(self) -> bool:
    return self.got_data

  def getTopic(self):
    return self.topic


class TimeDiffNowNode(Node):

  def __init__(self, topics):
    super().__init__('time_diff_now_node')
    self.timer = self.create_timer(0.5, self.timer_callback)
    self.sub_handles = []
    #create a handle for each topic
    for i in range(len(topics)):
      print("create topic: ", topics[i])
      if topics[i] == 'tf':
        print("create tf topic")
        self.tf_handle = TimeDiffHandleTf(self, topics[i])
      else:
        self.sub_handles.append(TimeDiffHandle(self, copy.deepcopy(topics[i])))



  def timer_callback(self):
    # self.get_logger().info('hans')
    invalid_idx = []
    # print("len handles: ", len(self.sub_handles))
    print('--')
   
    for i in range(len(self.sub_handles)):
      if not self.sub_handles[i].isValid():
        invalid_idx.append(i)
        print("got invalid idx")
        print("invalid idx: ", invalid_idx)
        continue
      time_diff = 'no data'
      if self.sub_handles[i].hasData():
        time_diff = self.sub_handles[i].getTimeDiff()
      print(self.sub_handles[i].getTopic(), ": ", time_diff, " s")
    for i in range(len(invalid_idx)):
      print('topic', self.sub_handles[invalid_idx[i]].getTopic() ,'is not valid -> this topic will be removed')
      self.sub_handles[invalid_idx[i]].clear()
      del self.sub_handles[invalid_idx[i]]

    # print('dings')
    if hasattr(self, 'tf_handle'):
      #print tf time diff
      self.tf_handle.printTfMessage()
      pass
      # print("++++++++topic: ", self.topic, "is not valid!!!")
    elif len(self.sub_handles) == 0:
      print("none of the given topics is valid.. will exit...")
      # rclpy.shutdown()
      exit(-1)
    print("-- --")

def main(args = None):

  num_args = len(sys.argv)
  if num_args <= 1:
    print('no topics given... will exit... use: ros2 run thi_tools time_diff_now_node <topic1> <topic2> <..>')
    exit(-1)
  tmp_args = copy.deepcopy(sys.argv)

  del tmp_args[0] #remove first element (contains path)

  rclpy.init(args=None)
  
  # print(args)

  node = TimeDiffNowNode(tmp_args)

  rclpy.spin(node)

  node.destroy_node()

  rclpy.shutdown()

if __name__ == '__main__':
  main()