import sqlite3
from typing import List
import pathlib
import rclpy
from sensor_msgs.msg import Image
from rclpy.publisher import Publisher
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
import numpy as np
import os
import utils

# trying to get topic from docker env, if TOPIC is not set => getting default topic name
TOPIC_NAME = os.environ.get("TOPIC", "/zed2i_back/zed_node/left/image_rect_color")
_debug = os.environ.get("DEBUG", True)


class ImageDataPublisher(Node):
    def __init__(self, db_path: pathlib.Path):
        super().__init__("lidar_data_publisher")

        self._publisher: Publisher = self.create_publisher(
            Image, "/sensor_msgs/msg/PointCloud", qos_profile_sensor_data
        )
        timer_period: float = 0.5
        self.db: sqlite3.Connection = sqlite3.connect(db_path)
        self.cursor: sqlite3.Cursor = self.db.cursor()
        self.i: int = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_timestamp: int = 0

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Create a float32array/pointcloud2 message
        msg = Image()
        header: Header = Header()
        stamp: Time = Time()

        timestamp, image_data = utils.getSingleMessageInTopic(
            self.cursor, TOPIC_NAME, self.last_timestamp
        )

        # divide timestamp to sec + nanosec
        stamp.sec = int(timestamp[:10])
        stamp.nanosec = int(timestamp[10:])
        header.stamp = stamp

        # Set the msg data
        msg.header = header
        msg.data = image_data

        # Publish the message to the topic
        self._publisher.publish(msg)

        # Display the message on the console
        self.get_logger().info(f"Publishing: {self.i}, timestamp - {header.stamp}")

        # Increment the counter by 1
        self.i += 1
        # Updating timstamp
        self.last_timestamp = int(timestamp)


if __name__ == "__main__":
    rclpy.init()
    db_path = pathlib.Path(
        "/home/amir/Desktop/sber/seqot-experiments/data/mental_test/2024_04_09_20_31_57_0.db3"
    )
    publisher = ImageDataPublisher(db_path)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()