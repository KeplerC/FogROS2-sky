
import atexit
import datetime
import json
import subprocess, os, yaml
import threading
import requests
import pprint
import socket 
import time 
import rclpy
import rclpy.node
import psutil
from rcl_interfaces.msg import SetParametersResult
from typing import List, Optional
from std_msgs.msg import Float64

from pydoc import locate

# NOTE: This must be the same as _SKY_REMOTE_BENCHMARK_DIR_SYMLINK
# in sky/benchmark/benchmark_utils.py.
_SKY_REMOTE_BENCHMARK_DIR = '~/sky_benchmark_dir'
# NOTE: This must be the same as _BENCHMARK_SUMMARY
# in sky/benchmark/benchmark_utils.py.
_BENCHMARK_SUMMARY = 'summary.json'

def get_ROS_class(ros_message_type, srv=False):
    """
    Returns the ROS message class from ros_message_type.
    :return AnyMsgClass: Class of the ROS message.
    """
    try:
        package_name, msg, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    if not srv:
        msg_class = locate('{}.msg.{}'.format(package_name, message_name))
    else:
        msg_class = locate('{}.srv.{}'.format(package_name, message_name))
    if msg_class is None:
        if srv:
            msg_or_srv = '.srv'
        else:
            msg_or_srv = '.msg'
        raise ValueError(
            'ros_message_type could not be imported. (' +
            ros_message_type + ', as "from ' + package_name +
            msg_or_srv + ' import ' + message_name + '" failed.')
    return msg_class

class HeuristicPubSub(rclpy.node.Node):
    def __init__(self):
        super().__init__('heuristic_pubsub')


        self.declare_parameter("whoami", "")
        self.identity = self.get_parameter("whoami").value

        self.declare_parameter("type", "")
        self.identity = self.get_parameter("type").value


        log_dir = _SKY_REMOTE_BENCHMARK_DIR
        log_dir = os.path.join(
            log_dir, 'sky-callback-' +
            datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f'))
        log_dir = os.path.expanduser(log_dir)
        os.makedirs(log_dir, exist_ok=True)


        # topic to subscribe to know the start and end of the benchmark
        self.declare_parameter("request_topic_name", "/gqcnn/image_compressed")
        # self.declare_parameter("request_topic_name", "/camera/color/image")
        request_topic = self.get_parameter("request_topic_name").value
        self.declare_parameter("request_topic_type", "sensor_msgs/msg/Image")
        request_topic_type = self.get_parameter("request_topic_type").value
        self.declare_parameter("response_topic_name", "/gqcnn/grasp_pose")
        # self.declare_parameter("response_topic_name", "/RGBD/pose")
        response_topic = self.get_parameter("response_topic_name").value
        self.declare_parameter("response_topic_type", "geometry_msgs/msg/PoseStamped")
        response_topic_type = self.get_parameter("response_topic_type").value
        self.declare_parameter("total_steps", 1000)
        total_steps = self.get_parameter("total_steps").value

        # last_response: use the time of the last response as the start time of the next step (dexnet, continuous)
        # first_request: use the time of the first request as the start time of the next step (slam, discrete)
        # user_define: user publish a timestamp to the topic /sky/latency
        self.declare_parameter("heuristic_mode", "last_response")
        # self.declare_parameter("heuristic_mode", "first_request")
        self.heuristic_mode = self.get_parameter("heuristic_mode").value        

        self.logger = self.get_logger()

        self.request_topic = self.create_subscription(
            get_ROS_class(request_topic_type),
            request_topic,
            self.request_topic_callback,
            1)

        self.response_topic = self.create_subscription(
            get_ROS_class(response_topic_type),
            response_topic,
            self.response_topic_callback,
            1)

        # TODO(woosuk): Do not store the entire timestamps.
        self._step_begins = []
        self._step_ends = []

        # Create a writer thread.
        self._worker = _AsyncSummaryWriter(log_dir, total_steps, 1,
                                           self._step_begins, self._step_ends)
        self._worker.start()
        self.logger.info("Latency analyzer started.")

        # The writer thread is a daemon thread, which is automatically killed
        # when the main process exits. The problem is, the training process
        # (and the writer daemon) can exit before the logs of the last few steps
        # are saved, because there is at most 1 sec (= save interval) time lag.
        # The purpose of this exit handler is to block the main process until
        # the daemon saves the up-to-date log and gracefully terminates.
        # Refer to: https://superfastpython.com/stop-daemon-thread/
        atexit.register(self._worker.stop)

        self.first_request_after_last_responded_time = None
    
    def request_topic_callback(self, msg):
        self.last_request_time = time.time()
        self.logger.info(f"request: {self.last_request_time}")
        if self.first_request_after_last_responded_time == None:
            self.first_request_after_last_responded_time = time.time()

    # calculate latency based on heuristics
    def response_topic_callback(self, msg):
        # float64 = Float64()
        # float64.data = (time.time() - self.last_request_time)
        # self.latency_publisher.publish(float64)

        if self.heuristic_mode == "first_request":
        # heuristic: use the time of the first request as the start time of the next step (slam, discrete)
            if self.first_request_after_last_responded_time == None:
                return 
            self._step_begins.append(self.first_request_after_last_responded_time)
            now = time.time()
            self._step_ends.append(now)
            self.logger.info(f"response: {time.time()}, {(time.time() - self.first_request_after_last_responded_time)}")
            self.first_request_after_last_responded_time = None
        # heuristic: use the time of the last response as the start time of the next step (dexnet, continuous)
        elif self.heuristic_mode == "last_response":
            now = time.time()
            self._step_ends.append(now)
            self._step_begins.append(now)
            self.logger.info(f"response: {time.time()}, {(now - self.first_request_after_last_responded_time)}")
            self.first_request_after_last_responded_time = now
        else:
            self.logger.error("not implemented yet")


# following code borrowed from https://github.com/KeplerC/skypilot/blob/7a53e0c0a66dbf2ca5df413b9b9206a9beceaf92/sky/callbacks/sky_callback/base.py#L73
# not important though
class _AsyncSummaryWriter(threading.Thread):
    class _BenchmarkSummary:

        def __init__(self,
                     boot_time: float,
                     create_time: float,
                     total_steps: Optional[int],
                     warmup_steps: int,
                     num_steps: int = 0,
                     first_step_time: Optional[float] = None,
                     warmup_end_time: Optional[float] = None,
                     last_step_time: Optional[float] = None,
                     time_per_step: Optional[float] = None,
                     estimated_total_time: Optional[float] = None) -> None:
            self.boot_time = boot_time
            self.create_time = create_time
            self.warmup_steps = warmup_steps
            self.total_steps = total_steps
            self.num_steps = num_steps
            self.first_step_time = first_step_time
            self.warmup_end_time = warmup_end_time
            self.last_step_time = last_step_time
            self.time_per_step = time_per_step
            self.estimated_total_time = estimated_total_time

    def __init__(self,
                 log_dir: str,
                 total_steps: Optional[int],
                 warmup_steps: int,
                 step_begins: List[float],
                 step_ends: List[float],
                 write_interval_seconds: float = 5) -> None:
        threading.Thread.__init__(self, daemon=True)
        self._log_path = os.path.join(log_dir, _BENCHMARK_SUMMARY)
        self._summary = self._BenchmarkSummary(
            boot_time=psutil.boot_time(),
            create_time=psutil.Process(os.getpid()).create_time(),
            total_steps=total_steps,
            warmup_steps=warmup_steps,
        )
        self._step_begins = step_begins
        self._step_ends = step_ends
        self._write_interval_seconds = write_interval_seconds

        # The thread will receive a stop signal when the main process exits,
        # so that it can save the up-to-date summary before termination.
        self._received_stop_signal = threading.Event()

    def stop(self) -> None:
        self._received_stop_signal.set()
        self.join()

    def _update_summary(self) -> None:
        summary = self._summary
        num_step_begins = len(self._step_begins)
        num_step_ends = len(self._step_ends)

        if summary.first_step_time is None:
            if num_step_begins > 0:
                summary.first_step_time = self._step_begins[0]
        if summary.warmup_end_time is None:
            if num_step_ends >= summary.warmup_steps:
                summary.warmup_end_time = self._step_ends[summary.warmup_steps -
                                                          1]
        num_steps = num_step_ends
        summary.num_steps = num_steps
        if num_steps > 0:
            last_step_time = self._step_ends[num_steps - 1]
            summary.last_step_time = last_step_time

        if num_steps > summary.warmup_steps:
            time_after_warmup = last_step_time - summary.warmup_end_time
            steps_after_warmup = num_steps - summary.warmup_steps
            time_per_step = time_after_warmup / steps_after_warmup
            summary.time_per_step = time_per_step

            if summary.total_steps is not None:
                # NOTE: total_time does not include the time
                # between booting and the process creation.
                time_until_warmup = summary.warmup_end_time - summary.create_time
                steps_after_warmup = summary.total_steps - summary.warmup_steps
                total_time = time_until_warmup + time_per_step * steps_after_warmup
                summary.estimated_total_time = total_time

    def _write_summary(self) -> None:
        with open(self._log_path, 'w') as f:
            json.dump(self._summary.__dict__, f)

    def run(self) -> None:
        """Periodically updates and saves the summary."""
        next_write_time = 0
        while not self._received_stop_signal.is_set():
            now = time.time()
            if now >= next_write_time:
                self._update_summary()
                self._write_summary()
                next_write_time = now + self._write_interval_seconds
            else:
                # Sleep for at most 1 second to avoid busy loop.
                time.sleep(min(1, next_write_time - now))

        # Update and save the summary one last time.
        self._update_summary()
        self._write_summary()

def main():
    rclpy.init()
    node = HeuristicPubSub()
    rclpy.spin(node)

if __name__ == '__main__':
    main()