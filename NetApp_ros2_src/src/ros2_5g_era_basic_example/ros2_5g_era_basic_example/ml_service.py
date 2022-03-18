# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
from email import header
import logging
import queue
import rclpy  # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import Executor
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ros_5g_era_object_detection.common import ThreadBase, get_logger as get_thread_logger
from concurrent.futures import ThreadPoolExecutor


from queue import Queue
import numpy as np
import json
import os

from std_msgs.msg import String

ROBOT_TOPIC_MAPPING = dict()

NUM_ROBOTS = 4

class PriorityExecutor(Executor):
    """
    Execute high priority callbacks in multiple threads, all others in a single thread.
    This is an example of a custom exectuor in python. Executors are responsible for managing
    how callbacks get mapped to threads. Rclpy provides two executors: one which runs all callbacks
    in the main thread, and another which runs callbacks in a pool of threads. A custom executor
    should be written if neither are appropriate for your application.
    """

    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.
        This is the only function which must be overridden by a custom executor. Its job is to
        start executing one callback, then return. It uses the method `wait_for_ready_callbacks`
        to get work to execute.
        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        """
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)


class ResultPublisher(Node):
    """
    Create an ResultPublisher class, which is a subclass of the Node class.
    """
    def __init__(self, robot_id, input_queue):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        node_name = robot_id+"_publisher"
        super().__init__(node_name)
        self.name = node_name
        self.robot_id = robot_id

        self.Q : Queue = input_queue


        # Create results publisher
        self.publisher = self.create_publisher(String, self.robot_id+'_results', 10)
        
        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """

        if not self.Q.empty():

            metadata, raw_results = self.Q.get()

            bbox, cls, score = raw_results
            results = dict()
            results["bbox"] = bbox
            results["class"] = cls
            results["score"] = score
            results["header"] = dict()

            stamp = metadata["header"].stamp
            results["header"]["stamp"] = "%d.%d" % (stamp.sec, stamp.nanosec)
            results["header"]["frame_id"] = metadata["header"].frame_id


            msg = String()
            msg.data = json.dumps(results)
            self.publisher.publish(msg)
            self.get_logger().info(f"ResultsPublisher: {self.name} - Publishing results: {results}")
    

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self, robot_id, output_queue):
        """
        Class constructor to set up the node
        """
        global ROBOT_TOPIC_MAPPING
        # Initiate the Node class's constructor and give it a name
        node_name = robot_id+"_subscriber"

        super().__init__(node_name)
        self.name = node_name
        self.robot_id = robot_id

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            self.robot_id + '_images',
            self.listener_callback,
            100)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Shared output queue
        self.Q = output_queue
        
        return 
    
    def listener_callback(self, data : Image):
        """
        Image callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)   

        # Create metadata for image
        metadata = {"node_name" : self.name,
                    "header": data.header}


        # Display the message on the console
        # self.get_logger().info('Receiving video frame')
        self.get_logger().info('Receiving image - frame_id: ' + data.header.frame_id)

        # Check if there is 
        if not self.Q.full():
            self.Q.put((metadata, current_frame))


class DummyDetector(ThreadBase):

    def __init__(self, logger, name, input_queue, output_queue):
        super().__init__(logger,name)

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.input_queue : Queue = input_queue
        self.output_queue : Queue = output_queue

    def _run(self):
        
        self.logger.info(f"{self.name} thread is running.")

        while True:

            if self.stopped:
                return

            # Check if data in input queue exists
            if self.input_queue.qsize() > 0:

                # Get image and metadata from input queue
                metadata, image = self.input_queue.get()
                img_h, img_w = image.shape[0:2]

                # Generate random bounding box
                max_bbox_size = 150
                bbox_x = np.random.randint(0,img_w-max_bbox_size)
                bbox_y = np.random.randint(0,img_h-max_bbox_size)
                bbox_w = np.random.randint(10,max_bbox_size)
                bbox_h = np.random.randint(10,max_bbox_size)
                bbox = [bbox_x, bbox_y, bbox_w, bbox_h]

                # Generate random class
                cls = np.random.randint(0,80)

                # Generate random detection score
                score = np.random.random()
                detection = (bbox, cls, score)
                
                # Put results in output queue
                self.output_queue.put((metadata,detection))


class QueueHandler(ThreadBase):

    def __init__(self, logger, name, input_queue, output_queues):
        super().__init__(logger,name)

        self.name = name
        self.Q : Queue = input_queue

        self.output_queues = output_queues

    def _run(self):

        while True:

            if self.stopped:
                return

            if not self.Q.empty():

                metadata, results = self.Q.get()

                robot_id = metadata["node_name"].split("_")[0]

                # Push to output queue based on "name" in metadata
                self.output_queues[robot_id].put((metadata,results))


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    logger = get_thread_logger(log_level=logging.INFO)

    queue_images = Queue(1024)
    queue_results = Queue(1024)

    detector_thread = DummyDetector(logger,"Detector",queue_images, queue_results)
    detector_thread.start(daemon = True)
    
    ros_nodes = []

    output_queues = dict()

    for i in range(NUM_ROBOTS):
        img_subscriber = ImageSubscriber("robot%d" % i, queue_images)
        ros_nodes.append(img_subscriber)


        pub_output_queue = Queue(1024)
        res_publisher = ResultPublisher("robot%d" % i, pub_output_queue)
        ros_nodes.append(res_publisher)

        output_queues["robot%d" % i] = pub_output_queue
        

    queue_handler_thread = QueueHandler(logger, "QueueHandler",queue_results, output_queues)
    queue_handler_thread.start(daemon = True)

    ros2_executor = PriorityExecutor() 
    for node in ros_nodes:
        ros2_executor.add_node(node)

    try:
        ros2_executor.spin()
    finally:
        ros2_executor.shutdown()
        for node in ros_nodes:
            node.destroy_node()


if __name__ == '__main__':
    main()
