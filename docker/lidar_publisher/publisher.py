import pathlib
import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, Image
from std_msgs.msg import Header
from rclpy.publisher import Publisher
from typing import List
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
import utils
import os


TOPIC_NAME = os.environ.get("TOPIC", "sensor_msgs/msg/PointCloud")
_debug = os.environ.get("DEBUG", True)


class LidarDataPublisher(Node):
    def __init__(self, dir_path: pathlib.Path):
        super().__init__("lidar_data_publisher")

        self._publisher: Publisher = self.create_publisher(
            PointCloud, TOPIC_NAME, qos_profile_sensor_data
        )
        timer_period: float = 0.5
        self.dir_path: pathlib.Path = dir_path
        self.i: int = 0
        self.data: List[str] = list()
        self.poses: List[np.ndarray] = list()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.read_file_names()

    def read_file_names(self) -> None:
        for file_name in sorted(self.dir_path.rglob("*.npy")):
            self.data.append(file_name.as_posix())
        print(len(self.data))

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Create a float32array/pointcloud2 message
        msg = PointCloud()
        header: Header = Header()
        stamp: Time = Time()

        timestamp, data = utils.getSingleMessageInTopic(
            self.cursor, TOPIC_NAME, self.last_timestamp
        )

        # divide timestamp to sec + nanosec
        stamp.sec = int(timestamp[:10])
        stamp.nanosec = int(timestamp[10:])
        header.stamp = stamp

        # Set the msg data
        msg.header = header
        msg.points = data

        # Set the msg data
        msg.header = header


        # Publish the message to the topic
        self._publisher.publish(msg)

        # Display the message on the console
        self.get_logger().info(f"Publishing: {self.i}, timestamp - {header.stamp}")

        # Increment the counter by 1
        self.i += 1


if __name__ == "__main__":
    rclpy.init()
    # info_dict = [
    #     {"help": "data source path", "type": str},
    # ]
    # parser = get_parser(["--path"], info_dict)
    # args = parser.parse_args()
    # path = pathlib.Path(args.path)
    if not _debug:
        publish = LidarDataPublisher(pathlib.Path("/data"))
    else:
        publish = LidarDataPublisher(pathlib.Path("./data/database/"))
    rclpy.spin(publish)
    publish.destroy_node()
    rclpy.shutdown()
