# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
# - src: https://automaticaddison.com/getting-started-with-opencv-in-ros-2-galactic-python/

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

import cv2  # OpenCV library
import os

from ros2_5g_era_helpers import get_path_to_assets

path_to_assets = get_path_to_assets()

IMG_PATH = os.path.join(path_to_assets, "test_image.jpg")
VIDEO_PATH = os.path.join(path_to_assets, "test_video.mp4")


class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(VIDEO_PATH)
        # self.img = cv2.imread(IMG_PATH)
        # cv2.imshow("test", self.img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Frame ID
        self.frame_id = 0

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_image_ = self.create_publisher(Image, 'images', 10)
        # self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            image_message = self.br.cv2_to_imgmsg(frame)
            image_message.header.stamp = self.get_clock().now().to_msg()
            image_message.header.frame_id = str(self.frame_id)
            self.publisher_image_.publish(image_message)

            self.frame_id += 1

            # Display the message on the console
            # self.get_logger().info('Publishing video frame')
            self.get_logger().info('Publishing image - frame_id: ' + image_message.header.frame_id)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher()

    try:
        # Spin the node so the callback function is called.
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in 5G-ERA ML Control Service:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        image_publisher.destroy_node()

        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == '__main__':
    main()
