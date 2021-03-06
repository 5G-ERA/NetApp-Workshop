# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

import json
import logging
import os
import uuid
import cv2
from concurrent.futures import ThreadPoolExecutor
from queue import Empty, Full, Queue
from typing import Dict  # for Python 3.9, just 'dict' will be fine

import numpy as np
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.executors import Executor
from rclpy.node import Node  # Handles the creation of nodes
from ros2_5g_era_helpers import get_path_to_assets
from ros2_5g_era_object_detection_standalone_py.common import ThreadBase, get_logger as get_thread_logger
from ros2_5g_era_service_interfaces.srv import Start, StateGet, StateReset, StateSet, Stop
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String

path_to_assets = get_path_to_assets()


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


class TaskHandler(Node):
    """
    Create an TaskHandler class, which is a subclass of the Node class.
    """

    def __init__(self, task_id: str, image_queue: "Queue"):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__(task_id)
        self._q = image_queue

        # Create results publisher
        self._result_publisher = self.create_publisher(String, f"/tasks/{self.get_name()}/results", 10)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self._subscription = self.create_subscription(
            Image,
            f"/tasks/{self.get_name()}/data",
            self.image_listener_callback,
            100)
        self._subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self._br = CvBridge()

        self.get_logger().info(f"TaskHandler node: {self.get_name()} created")

    def image_listener_callback(self, data: Image):
        """
        Image callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self._br.imgmsg_to_cv2(data)

        # Create metadata for image
        metadata = {"node_name": self.get_name(), "header": data.header}

        try:
            self._q.put((metadata, current_frame), block=False)  # TODO maybe add a little timeout here?
        except Full:
            self.get_logger().warning(f"Image queue full, skipping data (frame_id {data.header.frame_id})...")
            return

        self.get_logger().info('Receiving image - frame_id: ' + data.header.frame_id)

    def publish_results(self, data):

        metadata, raw_results = data

        # TODO this should definitely be a ROS message, not dict
        results = dict()
        results["header"] = dict()
        stamp = metadata["header"].stamp
        results["header"]["stamp"] = "%d.%d" % (stamp.sec, stamp.nanosec)
        results["header"]["frame_id"] = metadata["header"].frame_id

        results["detections"] = []

        for (bbox, cls, score) in raw_results:
            det = dict()
            det["bbox"] = [int(i) for i in bbox]
            det["class"] = int(cls)
            det["score"] = float(score)

            results["detections"].append(det)
        
        msg = String()
        msg.data = json.dumps(results)
        try:
            self._result_publisher.publish(msg)
            self.get_logger().info(f"ResultsPublisher: {self.get_name()} - Publishing results: {results}")
        except rclpy.handle.InvalidHandle:
            self.get_logger().info(f"ResultsPublisher: Node does not exists anymore. Skipping.")



TaskNodesDict = Dict[str, TaskHandler]


class FaceDetector(ThreadBase):

    def __init__(self, logger, name, input_queue, tasks: TaskNodesDict):
        super().__init__(logger, name)

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.input_queue: Queue = input_queue
        self._tasks = tasks
        self.detection_cascade = cv2.CascadeClassifier(os.path.join(path_to_assets, 'haarcascade_frontalface_default.xml'))

    def _run(self):

        self.logger.info(f"{self.name} thread is running.")

        while not self.stopped:

            # Get image and metadata from input queue
            try:
                metadata, image = self.input_queue.get(timeout=1)
            except Empty:
                continue

            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # gray = cv2.resize(gray,(640,360))
            # Detect the faces
            faces = self.detection_cascade.detectMultiScale(gray, 1.35, 4)

            detections = []
            
            for bbox in faces:

                # Generate random class
                cls = 1  # np.random.randint(0, 80)

                # Generate random detection score
                score = np.random.random()
                det = bbox, cls, score

                # Add to other detections for processed frame
                detections.append(det)

            # Put results in output queue
            task_id = metadata["node_name"]
            try:
                self._tasks[task_id].publish_results((metadata, detections))
            except KeyError:  # maybe the task was stopped in the meantime
                self.logger.warning(f"Task_id {task_id} no longer exists.")


class DummyDetector(ThreadBase):

    def __init__(self, logger, name, input_queue, tasks: TaskNodesDict):
        super().__init__(logger, name)

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.input_queue: Queue = input_queue
        self._tasks = tasks

    def _run(self):

        self.logger.info(f"{self.name} thread is running.")

        while not self.stopped:

            # Get image and metadata from input queue
            try:
                metadata, image = self.input_queue.get(timeout=1)
            except Empty:
                continue

            img_h, img_w = image.shape[0:2]

            # Generate random bounding box
            max_bbox_size = 150
            bbox_x = np.random.randint(0, img_w - max_bbox_size)
            bbox_y = np.random.randint(0, img_h - max_bbox_size)
            bbox_w = np.random.randint(10, max_bbox_size)
            bbox_h = np.random.randint(10, max_bbox_size)
            bbox = [bbox_x, bbox_y, bbox_w, bbox_h]

            # Generate random class
            cls = np.random.randint(0, 80)

            # Generate random detection score
            score = np.random.random()
            detection = (bbox, cls, score)

            # Put results in output queue
            task_id = metadata["node_name"]
            try:
                self._tasks[task_id].publish_results((metadata, detection))
            except KeyError:  # maybe the task was stopped in the meantime
                self.logger.warning(f"Task_id {task_id} no longer exists.")


class ControlService(Node):

    def __init__(self, executor: Executor, robot_nodes: TaskNodesDict, image_queue) -> None:
        super().__init__('ml_control_services')

        self._executor = executor
        self._robot_nodes = robot_nodes
        self._image_queue = image_queue

        self.start_srv = self.create_service(Start, f"~/{self.start.__name__}", self.start)
        self.stop_srv = self.create_service(Stop, f"~/{self.stop.__name__}", self.stop)
        self.state_set_srv = self.create_service(StateSet, "~/state/set", self.state_set)
        self.state_get_srv = self.create_service(StateGet, "~/state/get", self.state_get)
        self.state_reset_srv = self.create_service(StateReset, "~/state/reset", self.state_reset)

        self.get_logger().info("Control Service running...")

    def start(self, request: Start.Request, response: Start.Response) -> Start.Response:

        task_id = f"t{uuid.uuid4().hex}"  # prefixed with "t" (task), as node name must not start with number
        th = TaskHandler(task_id, self._image_queue)
        self._robot_nodes[task_id] = th
        self._executor.add_node(th)

        self.get_logger().info(f"Starting task_id: {task_id}, worker settings: {request.robot_worker_setting}.")
        response.success = True
        response.task_id = task_id

        # TODO maybe that returning topic names is redundant?
        response.data_topic = f"/tasks/{task_id}/data"
        response.result_topic = f"/tasks/{task_id}/results"

        return response

    def stop(self, request: Stop.Request, response: Stop.Response) -> Stop.Response:

        try:
            th = self._robot_nodes[request.task_id]
        except KeyError:
            response.success = False
            response.message = "Unknown task_id."
            return response

        th.destroy_node()
        self._executor.remove_node(th)

        self.get_logger().info(f"Stopping task_id: {request.task_id}.")

        response.success = True

        return response

    def state_set(self, request: StateSet.Request, response: StateSet.Response) -> StateSet.Response:
        response.success = False
        response.message = "Not implemented"
        return response

    def state_get(self, request: StateGet.Request, response: StateGet.Response) -> StateGet.Response:
        response.success = False
        response.message = "Not implemented"
        return response

    def state_reset(self, request: StateReset.Request, response: StateReset.Response) -> StateReset.Response:
        response.success = False
        response.message = "Not implemented"
        return response


def main(args=None):
    task_nodes: TaskNodesDict = dict()  # there is a node for each created task
    image_queue = Queue(1024)  # buffer of images for the detector

    # Initialize the rclpy library
    rclpy.init(args=args)

    logger = get_thread_logger(log_level=logging.INFO)

    detector_thread = FaceDetector(logger, "Detector", image_queue, task_nodes)
    detector_thread.start(daemon=True)

    if __debug__:
        # this is useful for debugging, because the process stops on any unhandled exception
        from rclpy.executors import SingleThreadedExecutor
        ros2_executor = SingleThreadedExecutor()  # PriorityExecutor()
    else:
        # this swallows exceptions (not sure why), which makes debugging hard...
        ros2_executor = PriorityExecutor()  # TODO maybe use MultiThreadedExecutor instead?

    ros2_executor.add_node(ControlService(ros2_executor, task_nodes, image_queue))

    try:
        ros2_executor.spin()
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in 5G-ERA ML Control Service:', file=sys.stderr)
        raise
    finally:
        ros2_executor.shutdown()
        for node in ros2_executor.get_nodes():
            node.destroy_node()


if __name__ == '__main__':
    main()
