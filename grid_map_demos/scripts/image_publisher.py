#!/usr/bin/env python3
# simple script to publish a image from a file.
import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('image_path')
        self.declare_parameter('topic')

        self.imagePath_ = self.get_parameter(
            'image_path').get_parameter_value().string_value
        topicName = self.get_parameter(
            'topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, topicName, 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        """Convert a image to a ROS compatible message (sensor_msgs.Image)."""
        img = cv2.imread(self.imagePath_, cv2.IMREAD_UNCHANGED)

        # print img.shape
        # print img.size
        # print img.dtype.itemsize

        rosimage = Image()

        if img.dtype.itemsize == 2:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr16'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra16'
            else:
                rosimage.encoding = 'mono16'
        if img.dtype.itemsize == 1:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr8'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra8'
            else:
                rosimage.encoding = 'mono8'
        # print('Encoding: ', rosimage.encoding)

        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0]
        rosimage.data = img.tostring()
        rosimage.header.stamp = self.get_clock().now().to_msg()
        rosimage.header.frame_id = 'map'

        self.publisher_.publish(rosimage)


# Main function initializes node and subscribers and starts the ROS loop
def main_program(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main_program()
