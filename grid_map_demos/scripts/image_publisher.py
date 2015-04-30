#!/usr/bin/env python
# simple script to publish a image from a file.
import rospy
import time
import cv2
import sensor_msgs.msg

#change these to fit the expected topic names
IMAGE_MESSAGE_TOPIC = 'grid_map_image'
IMAGE_PATH = 'bgra8.png'

def callback(self):
    """ Convert a image to a ROS compatible message
        (sensor_msgs.Image).
    """
    img = cv2.imread(IMAGE_PATH, -1)

    rosimage = sensor_msgs.msg.Image()
    rosimage.encoding = 'bgra8'
    rosimage.width = img.shape[1]
    rosimage.height = img.shape[0]
    rosimage.step = img.strides[0]
    rosimage.data = img.tostring()
    rosimage.header.stamp = rospy.Time.now()
    rosimage.header.frame_id = ''
#    rosimage.data = img.flatten().tolist()

    publisher.publish(rosimage)


#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher
    rospy.init_node('image_publisher')
    publisher = rospy.Publisher(IMAGE_MESSAGE_TOPIC, sensor_msgs.msg.Image, queue_size=10)
    rospy.Timer(rospy.Duration(2), callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
