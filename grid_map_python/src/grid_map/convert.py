import numpy as np
from grid_map_python import GridMapBinding

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap as GridMapMsg

def from_msg(msg, cls=GridMapBinding):
  gm = cls()
  gm.setFrameId(msg.info.header.frame_id)
  gm.setTimestamp(int(msg.info.header.stamp.to_sec()*10E9))
  gm.setStartIndex(np.array((msg.outer_start_index, msg.inner_start_index)))
  gm.setGeometry(
    np.array((msg.info.length_x, msg.info.length_y)),
    msg.info.resolution,
    np.array((msg.info.pose.position.x, msg.info.pose.position.y))
  )

  for i, layer in enumerate(msg.layers):
    gm.add(layer, np.float32(np.reshape(msg.data[i].data, (msg.data[i].layout.dim[1].size, msg.data[i].layout.dim[0].size), order='F')) )

  return gm

def to_msg(self):
  msg = GridMapMsg()
  msg.info.header.stamp = rospy.Time(self.getTimestamp()/10E9)
  msg.info.header.frame_id = self.getFrameId()
  msg.info.resolution = self.getResolution()
  msg.info.length_x = self.getLength()[0]
  msg.info.length_y = self.getLength()[1]
  msg.info.pose.position.x, msg.info.pose.position.y = self.getPosition()
  msg.info.pose.orientation.w = 1
  msg.layers = list(self.getLayers())
  for layer in self.getLayers():
    matrix = self[layer]
    data_array = Float32MultiArray()
    data_array.layout.dim.append(MultiArrayDimension("column_index", matrix.shape[1], matrix.shape[0]*matrix.shape[1]))
    data_array.layout.dim.append(MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0]))
    data_array.data = matrix.flatten(order='F')
    msg.data.append(data_array)

  return msg
