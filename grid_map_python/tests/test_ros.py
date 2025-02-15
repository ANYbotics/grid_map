#!/usr/bin/env python3

import rospy
import numpy as np
from grid_map import GridMap, from_msg, to_msg
from grid_map_msgs.msg import GridMap as GridMapMsg

class GridMapPythonTest_ROS:
  def __init__(self) :
    self.pub = rospy.Publisher('out', GridMapMsg, queue_size=0)
    self.sub = rospy.Subscriber('in', GridMapMsg, self.callback)

  def callback(self, msg):
    gm = from_msg(msg)
    msgOut = to_msg(gm)
    msgOut.info.header.seq = msg.info.header.seq
    self.pub.publish(msgOut)

if __name__ == '__main__':
  try:
    rospy.init_node('GridMapPythonTest_ROS')
    node = GridMapPythonTest_ROS()
    rospy.spin()
  except rospy.ROSInterruptException as e:
    pass
