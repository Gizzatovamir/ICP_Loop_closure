import rclpy
from rclpy.node import Node
from typing import Union
from sensor_msgs.msg import PointCloud2, Image
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import message_filters
from rclpy.publisher import Publisher
import argparse
import pathlib
import os

IMAGE_TOPIC = os.environ.get(
    "IMAGE_TOPIC",
)
LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "sensor_msgs/msg/PointCloud2")
RESULT_TOPIC = os.environ.get("RESULT_TOPIC", "depth_image")


class ResultPublisher(Node):
    def __init__(self):
        self.publisher: Publisher = self.create_publisher(Image, RESULT_TOPIC, 10)

    def publish_callback(self, msg1: PointCloud2, msg2: Image):
        self.get_logger().info(
            f"Got msgs - {msg1.header.stamp}, {msg2.header.stamp}, publishing result"
        )
        self.publisher.publish(msg2)


class DataSubscriber(Node):
    def __init__(self):
        super().__init__("data_listener")
        self.pose_subscriber = message_filters.Subscriber(
            self,
            Image,
            IMAGE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.lidar_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            LIDAR_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync_publishers = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_subscriber, self.pose_subscriber],
            1,
            slop=0.5,
            allow_headerless=False,
        )
        self.sync_publishers.registerCallback(self.test)
        self.publisher: ResultPublisher = ResultPublisher()

    def test(self, msg1: PointCloud2, msg2: Image):
        self.get_logger().info(f"I heard about point cloud: {msg1.header.stamp}")
        self.get_logger().info(f"I heard about pose: {msg2.header.stamp}")
        self.publisher.publish_callback(msg1, msg2)


if __name__ == "__main__":
    rclpy.init()
    my_subscriber = DataSubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()
