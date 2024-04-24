import rclpy
from rclpy.node import Node
from typing import Union
from sensor_msgs.msg import PointCloud, Image
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import message_filters
import pathlib
from LoopClosure import LoopClosure
import os

POSE_TOPIC = os.environ.get("POSE_TOPIC", "/geometry_msgs/msg/PoseStamped")


class DataSubscriber(Node):
    def __init__(self):
        super().__init__("place_recognition_sub")
        self.pose_subscriber = message_filters.Subscriber(
            self,
            PoseStamped,
            POSE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.lidar_subscriber = message_filters.Subscriber(
            self,
            PointCloud,
            "/sensor_msgs/msg/PointCloud",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync_publishers = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_subscriber, self.pose_subscriber],
            1,
            slop=0.5,
            allow_headerless=False,
        )
        self.sync_publishers.registerCallback(self.test)

    def test(self, msg1: PointCloud, msg2: PoseStamped):
        self.get_logger().info(f"I heard about point cloud: {msg1.header.stamp}")
        self.get_logger().info(f"I heard about pose: {msg2.header.stamp}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--cfg", help="path to config")
    args = parser.parse_args()
    config: dict = yaml.safe_load(open(args.cfg))
    seqlen: int = int(config["gen_sub_descriptors"]["seqlen"])
    pretrained_weights: pathlib.Path = pathlib.Path(
        config["test_gem_prepare"]["weights"]
    )
    feature_ex_pretrained_weights: pathlib.Path = pathlib.Path(
        config["gen_sub_descriptors"]["weights"]
    )
    range_image_database_root: pathlib.Path = pathlib.Path(
        config["data_root"]["range_image_database_root"]
    )
    descriptors_path: pathlib.Path = pathlib.Path(
        config["test_gem_prepare"]["sub_descriptors_database_file"]
    )
    rclpy.init()
    # print(inspect.getfile(message_filters.ApproximateTimeSynchronizer([], 1,1).__class__))
    my_subscriber = DataSubscriber(
        db_path=range_image_database_root,
        gem_pretrained_weights=pretrained_weights,
        feature_extracter_weights=feature_ex_pretrained_weights,
        descriptors_path=descriptors_path,
    )

    rclpy.spin(my_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_subscriber.destroy_node()
    rclpy.shutdown()
