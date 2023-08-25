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
  gm.setBasicLayers(list(msg.basic_layers))
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
  msg.basic_layers=list(self.getBasicLayers())
  for layer in self.getLayers():
    matrix = self[layer]
    data_array = Float32MultiArray()
    data_array.layout.dim.append(MultiArrayDimension("column_index", matrix.shape[1], matrix.shape[0]*matrix.shape[1]))
    data_array.layout.dim.append(MultiArrayDimension("row_index", matrix.shape[0], matrix.shape[0]))
    data_array.data = matrix.flatten(order='F')
    msg.data.append(data_array)

  return msg
  
#Converts OccupancyGrid Message to GridMapMsg 
def from_OccupancyGrid(map1,cls=GridMapBinding):
    msg = GridMapMsg()
    msg.info.header.stamp=map1.header.stamp
    msg.info.header.frame_id=map1.header.frame_id
    msg.info.resolution=map1.info.resolution
    msg.info.length_x=map1.info.width*map1.info.resolution
    msg.info.length_y=map1.info.height*map1.info.resolution
    
    #Next step is due to difference in center definition
    #OccupancyGrid defines coordinate (0,0) as the location of map frame in real world. Occupancy grid defines cell [0,0] as the cell with the most negative x and y value
    #GridMapMsg defines coordinate (0,0) as the center of the grid map generated. Grid Map defines cell [0,0] as the cell with the most positive x and y value
    msg.info.pose.position.x=0.5*msg.info.length_x+map1.info.origin.position.x 
    msg.info.pose.position.y=0.5*msg.info.length_y+map1.info.origin.position.y
    msg.info.pose.position.z=0.0 #Can be any value
    
    msg.info.pose.orientation.x=0.0
    msg.info.pose.orientation.y=0.0
    msg.info.pose.orientation.z=0.0
    msg.info.pose.orientation.w=1.0
    msg.layers=['grid']
    msg.basic_layers=['grid']
    data_array=Float32MultiArray()
    data_array.layout.dim.append(MultiArrayDimension("column_index", map1.info.height, map1.info.height*map1.info.width)) #length_y is coloumn index is height
    data_array.layout.dim.append(MultiArrayDimension("row_index", map1.info.width, map1.info.width)) #length_x is row index is width
    data_array.data = map1.data
    #inverse iteration due to difference in convention of grid_map_msg/GridMapMsg and nav_msgs/OccupancyGrid
    data_array.data = data_array.data[::-1] 
    msg.data.append(data_array)
    
    return msg
