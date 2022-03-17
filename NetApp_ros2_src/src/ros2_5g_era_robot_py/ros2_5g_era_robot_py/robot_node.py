import argparse
import os.path
from argparse import ArgumentTypeError
import sys
import time
from typing import Tuple, Union
import json
import cv2  # OpenCV library.
import numpy as np
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from std_msgs.msg import String
from sensor_msgs.msg import Image

from ros2_5g_era_helpers import get_path_to_assets

from ros2_5g_era_robot_interfaces.srv import StopService
from ros2_5g_era_robot_interfaces.srv import StartService

from ros2_5g_era_service_interfaces.srv import Start
from ros2_5g_era_service_interfaces.srv import Stop

path_to_assets = get_path_to_assets()

IMG_PATH = os.path.join(path_to_assets, "test_image.jpg")
VIDEO_PATH = os.path.join(path_to_assets, "test_video.mp4")
USE_VIDEO = True

# Default run time (in seconds).
DEFAULT_RUN_TIME = 0


def positive_or_zero_int(string: str) -> int:
    """
    Check and convert string to positive or zero int.

    :param string: String to check.
    :return: Converted string as int.
    """
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be a positive or zero integer')
    return value


class RobotMLControlServicesClient(Node):
    """
    RobotMLControlServicesClient class, which is a subclass of the Node class.
    The node provides communication with 5G-ERA ML Control Services.
    """

    def __init__(self, service_base_name: str):
        """
        Class constructor to set up the node.
        """
        super().__init__('robot_ml_control_services_client')

        # Create clients for 5G-ERA ML Control Services.
        self.start_service_client = self.create_client(Start, service_base_name + '/start')
        self.get_logger().info(service_base_name + '/start client created')
        self.stop_service_client = self.create_client(Stop, service_base_name + '/stop')
        self.get_logger().info(service_base_name + '/stop client created')

        # Wait for the services' connection.
        attempts = 0
        while attempts < 5 and not self.start_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('5G-ERA ML Control Service "Start" is not available, waiting again ...')
            attempts += 1
        if not self.start_service_client.service_is_ready():
            raise ValueError(
                '5G-ERA ML Control Service "Start" with base name "' + service_base_name + '" is not available')
        self.get_logger().info(service_base_name + '/start client connected')

        attempts = 0
        while attempts < 5 and not self.stop_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('5G-ERA ML Control Service "Stop" is not available, waiting again ...')
            attempts += 1
        if not self.stop_service_client.service_is_ready():
            raise ValueError(
                '5G-ERA ML Control Service "Stop" with base name "' + service_base_name + '" is not available')
        self.get_logger().info(service_base_name + '/stop client connected')

        # Future and response variables.
        self.start_request_future = None
        self.start_request_response = None
        self.stop_request_future = None
        self.stop_request_response = None

        self.get_logger().info('robot_ml_control_services_client node is running')

    def send_start_request(self) -> bool:
        """
        Send "Start" 5G-ERA ML Control Service request.

        :return: True if the start_service_client is ready and the request was sent, otherwise returns False.
        """
        # Cancel any 5G-ERA ML Control Service unfinished requests.
        if self.stop_request_future and not self.stop_request_future.done():
            self.stop_request_future.cancel()
        if self.start_request_future and not self.start_request_future.done():
            self.start_request_future.cancel()
        # Create new Start service request.
        start_request = Start.Request()
        start_request.robot_worker_setting = ""
        # Delete old responses.
        self.start_request_response = None
        self.stop_request_response = None
        # Send "Start" 5G-ERA ML Control Service request.
        if self.start_service_client.service_is_ready():
            self.start_request_future = self.start_service_client.call_async(start_request)
            self.start_request_future.add_done_callback(self.start_request_future_done_callback)
            return True
        else:
            return False

    def send_stop_request(self) -> bool:
        """
        Send "Stop" 5G-ERA ML Control Service request.

        :return: True if the stop_service_client is ready and the request was sent, otherwise returns False.
        """
        # Cancel any 5G-ERA ML Control Service unfinished requests.
        if self.stop_request_future and not self.stop_request_future.done():
            self.stop_request_future.cancel()
        if self.start_request_future and not self.start_request_future.done():
            self.start_request_future.cancel()
        # Create new Stop service request.
        stop_request = Stop.Request()
        stop_request.task_id = self.start_request_response.task_id
        # Delete old responses.
        self.start_request_response = None
        self.stop_request_response = None
        # Send "Stop" 5G-ERA ML Control Service request.
        if self.stop_service_client.service_is_ready():
            self.stop_request_future = self.stop_service_client.call_async(stop_request)
            self.stop_request_future.add_done_callback(self.stop_request_future_done_callback)
            return True
        else:
            return False

    def start_request_future_done_callback(self, future: Future) -> None:
        """
        "Start" 5G-ERA ML Control Service request Future done callback.

        :param future: Future object of request.
        """
        if future.done():
            try:
                self.start_request_response = future.result()
            except Exception as e:
                self.start_request_response = None
                self.get_logger().info(
                    '5G-ERA ML Control Service call failed %r.' % (e,))
            else:
                self.get_logger().info(
                    'Result of "Start" 5G-ERA ML Control Service request: \n '
                    'success: %r, message: "%s", task_id: %s, \n '
                    'data_topic: "%s", result_topic: "%s", worker_constraints: "%s"' %
                    (self.start_request_response.success, self.start_request_response.message,
                     self.start_request_response.task_id,
                     self.start_request_response.data_topic, self.start_request_response.result_topic,
                     self.start_request_response.worker_constraints))

    def stop_request_future_done_callback(self, future: Future) -> None:
        """
        "Stop" 5G-ERA ML Control Service request Future done callback.

        :param future: Future object of request.
        """
        if future.done():
            try:
                self.stop_request_response = future.result()
            except Exception as e:
                self.stop_request_response = None
                self.get_logger().info(
                    '5G-ERA ML Control Service call failed %r.' % (e,))
            else:
                self.get_logger().info(
                    'Result of "Stop" 5G-ERA ML Control Service request: \n '
                    'success: %r, message: "%s"' %
                    (self.stop_request_response.success, self.stop_request_response.message))

    def get_topic_names(self) -> Union[Tuple[str, str], Tuple[None, None]]:
        """
        Try to get the topic name if "Start" 5G-ERA ML Control Service has already started and responded.

        :return: Data topic name, Result topic name.
        """
        if self.start_request_response:
            return self.start_request_response.data_topic, self.start_request_response.result_topic
        else:
            return None, None

    def service_is_running(self) -> bool:
        """
        Is the 5G-ERA ML Control Service in running state?

        :return: True if the service is running, otherwise returns False.
        """
        if self.start_request_response:
            return True
        else:
            return False


class RobotLogic(Node):
    """
    RobotLogic class, which is a subclass of the Node class.
    The node represents the logic of the robot.
    """

    def __init__(self, node_name: str = 'robot_logic'):
        """
        Class constructor to set up the node.
        """
        super().__init__(node_name)
        self.robot_ml_control_services_client = None
        self.publisher = None
        self.subscription = None

        # Create services for middleware communication.
        self.start_service = self.create_service(StartService, node_name + '/start_service',
                                                 self.start_service_callback)
        self.get_logger().info(node_name + '/start_service is running')
        self.stop_service = self.create_service(StopService, node_name + '/stop_service', self.stop_service_callback)
        self.get_logger().info(node_name + '/stop_service is running')

        # Create the timer.
        self.timer = self.create_timer(0.1, self.image_publisher_callback)

        # Create a VideoCapture object.
        # The argument '0' gets the default webcam.
        # self.cap = cv2.VideoCapture(0)
        self.img = cv2.imread(IMG_PATH)
        # cv2.imshow("test", self.img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        if USE_VIDEO:
            # Create a VideoCapture object.
            self.video_capture = cv2.VideoCapture(VIDEO_PATH)
        else:
            self.img = cv2.imread(IMG_PATH)

        # Store published images for further processing
        self.image_storage = dict()

        # Frame ID.
        self.frame_id = 0

        # Used to convert between ROS and OpenCV images.
        self.br = CvBridge()
        self.get_logger().info(node_name + ' node is running')

    def start_service_callback(self, request: StartService.Request,
                               response: StartService.Response) -> StartService.Response:
        """
        Start service callback.

        :param request: Service request with 5G-ERA ML Control Services base name.
        :param response: StartService Response.
        :return: StartService Response.
        """
        if not self.robot_ml_control_services_client:
            # Try to create the RobotMLControlServicesClient node.
            try:
                self.robot_ml_control_services_client = RobotMLControlServicesClient(request.service_base_name)
                self.executor.add_node(self.robot_ml_control_services_client)
                response.message = 'Robot Service Client successfully created and connected with 5G-ERA ML Control Services (' + request.service_base_name + '). '
            except ValueError as e:
                response.success = False
                response.message = str(e)
                return response

        if self.robot_ml_control_services_client and not self.robot_ml_control_services_client.service_is_running():
            # Call start service.
            if self.robot_ml_control_services_client.send_start_request():
                response.success = True
                response.message += 'Robot Service Client successfully sent "Start" request to 5G-ERA ML Control Services'
            else:
                response.success = False
                response.message += '5G-ERA ML Control Service "Start" is not available'
        else:
            response.success = False
            response.message = '5G-ERA ML Control Services are already in a running state'
        return response

    def stop_service_callback(self, request: StopService.Request,
                              response: StopService.Response) -> StopService.Response:
        """
        Stop service callback.

        :param request: Service request with 5G-ERA ML Control Services base name, Unused in this example.
        :param response: StopService Response.
        :return: StopService Response.
        """
        if not self.robot_ml_control_services_client:
            response.success = False
            response.message = 'Robot Service Client has not been created yet'
            return response

        if self.robot_ml_control_services_client and self.robot_ml_control_services_client.service_is_running():
            # Call stop service.
            if self.robot_ml_control_services_client.send_stop_request():
                response.success = True
                response.message = 'Robot Service Client successfully sent "Stop" request to 5G-ERA ML Control Services'
            else:
                response.success = False
                response.message = '5G-ERA ML Control Service "Stop" is not available'
        else:
            response.success = False
            response.message = '5G-ERA ML Control Services is not in running state'
        return response

    def image_publisher_callback(self) -> None:
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        frame = self.img
        image_message = self.br.cv2_to_imgmsg(self.img)

        if USE_VIDEO:
            # Capture frame-by-frame as the video frame.
            ret, frame = self.video_capture.read()
            # Create image message.
            if ret:
                image_message = self.br.cv2_to_imgmsg(frame)
            else:
                self.get_logger().info('Video is not available')
                if self.robot_ml_control_services_client and self.robot_ml_control_services_client.service_is_running():
                    # Call stop service.
                    if self.robot_ml_control_services_client.send_stop_request():
                        self.get_logger().info('Robot Service Client successfully sent "Stop" request to 5G-ERA ML Control Services')
                    else:
                        self.get_logger().info('5G-ERA ML Control Service "Stop" is not available')
                self.timer.cancel()

        if not self.publisher and not self.subscription:
            # Try to get created topic name.
            if self.robot_ml_control_services_client:
                data_topic, result_topic = self.robot_ml_control_services_client.get_topic_names()
                if data_topic and result_topic:
                    # Create image publisher with a given topic name.
                    self.subscription = self.create_subscription(String, result_topic, self.result_callback, 10)
                    self.publisher = self.create_publisher(Image, data_topic, 100)
                    self.get_logger().info('Publisher on topic "%s" and subscription on topic "%s" created' % (
                        data_topic, result_topic))
        else:
            if self.robot_ml_control_services_client and not self.robot_ml_control_services_client.service_is_running():
                self.get_logger().info('Publisher on topic "%s" and subscription on topic "%s" destroyed' % (
                    self.publisher.topic_name, self.subscription.topic_name))
                self.destroy_subscription(self.subscription)
                self.subscription = None
                self.destroy_publisher(self.publisher)
                self.publisher = None
            else:
                # Set frame id.
                image_message.header.frame_id = str(self.frame_id)
                image_message.header.stamp = self.get_clock().now().to_msg()
                self.frame_id += 1
                # Publish image message.
                self.publisher.publish(image_message)

                # Store image internally
                frame_key = "%s.%s" % (image_message.header.stamp.sec, image_message.header.stamp.nanosec)
                self.image_storage[frame_key] = frame

                # Display the message on the console.
                self.get_logger().info('Publishing image %s' % image_message.header.frame_id)

    def result_callback(self, msg) -> None:
        """
        Callback for received results.

        :param msg: Incoming message.
        """
        self.get_logger().info('I heard: %s' % msg.data)

        # Load JSON.
        data = json.loads(msg.data)
        header = data["header"]
        # Get stored frame.
        frame = self.image_storage[header["stamp"]]
        # Load detections.
        detections = data["detections"]

        for d in detections:
            x, y, w, h = d["bbox"]
            cls = d["class"]
            score = d["score"]
            # Draw detection into frame.
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("Detection_results", frame)
        cv2.waitKey(1)


def main(args=None):
    """Main function."""
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description='ROS2 5G-ERA robot testing app')
    parser.add_argument('--run_time', '-r', dest='run_time', type=positive_or_zero_int, action='store',
                        help='run time (0 means that it will run until it is interrupted', default=DEFAULT_RUN_TIME)
    parser.add_argument('--robot_node_name', '-n', dest='robot_node_name', action='store',
                        help='Robot logic node name', default='robot_logic')
    arguments = parser.parse_args()

    # Initialize the rclpy library.
    rclpy.init(args=args)

    # Create executor for multiple nodes.
    executor = None
    if __debug__:
        # this is useful for debugging, because the process stops on any unhandled exception
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()  # PriorityExecutor()
    else:
        # this swallows exceptions (not sure why), which makes debugging hard...
        executor = MultiThreadedExecutor()

    # Create the RobotLogic node.
    robot_logic = RobotLogic(arguments.robot_node_name)
    executor.add_node(robot_logic)

    try:
        if arguments.run_time == 0:
            # Keep publishing.
            executor.spin()
        else:
            # Publish messages for a set run time.
            t_end = time.time() + arguments.run_time
            while time.time() < t_end:
                executor.spin_once()
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in robot:', file=sys.stderr)
        raise
    finally:
        executor.shutdown()
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
