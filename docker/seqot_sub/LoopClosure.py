from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
from typing import List, Tuple
from rclpy.duration import Duration
import numpy as np
import pathlib
import torch
from models import GeM, featureExtracter
import faiss

EVAL_PATH = pathlib.Path("/data/test_data")


def get_np_array_from_path(path: pathlib.Path) -> List[np.ndarray]:
    """
        Method to get list of ndarrays of range images to generate descriptor with Gem in validation class
    Args:
        path (pathlib.Path): path to dir with validating images

    Returns:
        list of ndarrays from validating dir
    """
    res: List[np.ndarray] = list()
    for data_path in path.rglob("*.npy"):
        np_data: np.ndarray = np.load(data_path)
        res.append(np_data)
    return res


def read_descriptors(index_in_db_dir, descriptors, seq_len) -> torch.Tensor:
    half_seq: int = int(seq_len // 2)
    start_index: int = index_in_db_dir - half_seq
    end_index: int = index_in_db_dir - half_seq + seq_len - 1
    descriptors_seq: torch.FloatTensor = (
        torch.zeros((1, seq_len, 256)).type(torch.FloatTensor).cuda()
    )
    for index in range(start_index, end_index):
        if index < len(descriptors):
            descriptor_tensor = (
                torch.from_numpy(descriptors[index, :]).type(torch.FloatTensor).cuda()
            )
            descriptors_seq[0, int(index - int(index_in_db_dir) + half_seq), :] = (
                descriptor_tensor
            )
        else:
            descriptor_tensor = (
                torch.from_numpy(descriptors[len(descriptors) - index, :])
                .type(torch.FloatTensor)
                .cuda()
            )
            descriptors_seq[0, int(index - int(index_in_db_dir) + half_seq), :] = (
                descriptor_tensor
            )
    return descriptors_seq


class LoopClosure:
    def __init__(
        self,
        db_path: pathlib.Path = pathlib.Path(""),
        descriptors_path: pathlib.Path = None,
        gem_pretrained_weights: pathlib.Path = None,
        feature_extracter_weights=None,
        slop=0.5,
        seqlen=10,
    ):
        self.point_cloud_queue: List[PointCloud] = list()
        self.pose_queue: List[PoseStamped] = list()
        self.slop: float = slop
        self.seqlen: int = seqlen
        self.descriptors: np.ndarray = np.load(descriptors_path.as_posix())
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.descriptors_db_path: pathlib.Path = db_path
        self.model: GeM = GeM().to(self.device)
        self.gem_weights: pathlib.Path = gem_pretrained_weights
        self.feature_ex_weights: pathlib.Path = feature_extracter_weights
        self.extracter_model: featureExtracter = featureExtracter().to(self.device)

    def add(self, point_cloud: PointCloud, pose: PoseStamped):
        duration = Duration(
            nanoseconds=abs(
                point_cloud.header.stamp.nanosec - pose.header.stamp.nanosec
            )
        )
        if duration.nanoseconds > self.slop:
            print(
                f"Difference between 2 msg timstamps are too big - {duration.nanoseconds}"
            )
            return
        else:
            self.point_cloud_queue.append(point_cloud)
            self.pose_queue.append(pose)

    def load_feature_extracter(self):
        checkpoint = torch.load(self.feature_ex_weights.as_posix())
        self.extracter_model.load_state_dict(checkpoint["state_dict"])
        self.extracter_model.eval()

    def gen_depth_data_seq(self, test_path: pathlib.Path) -> torch.FloatTensor:
        range_image_list = get_np_array_from_path(test_path)
        depth_data_seq: torch.FloatTensor = (
            torch.zeros((1, len(range_image_list), 32, 900))
            .type(torch.FloatTensor)
            .cuda()
        )
        for index, depth_data in enumerate(range_image_list):
            depth_data_tensor = (
                torch.from_numpy(depth_data).type(torch.FloatTensor).cuda()
            )
            depth_data_tensor = torch.unsqueeze(depth_data_tensor, dim=0)
            depth_data_tensor = torch.unsqueeze(depth_data_tensor, dim=0)
            depth_data_seq[:, index, :, :] = depth_data_tensor
        return depth_data_seq

    def create_value_seq(self, test_path: pathlib.Path) -> np.ndarray:
        self.load_feature_extracter()
        depth_data_seq = self.gen_depth_data_seq(test_path)
        des_list: torch.FloatTensor = (
            torch.zeros((1, len(depth_data_seq), 256)).type(torch.FloatTensor).cuda()
        )
        for index, depath_data in enumerate(depth_data_seq):
            current_batch_des = self.extracter_model(depth_data_seq)
            current_batch_des = current_batch_des.squeeze(1)
            des_list[index] = current_batch_des
        del self.feature_ex_weights
        # self.feature_ex_weights = featureExtracter().to(self.device)
        return self.model(des_list).squeeze(1)[0, :].cpu().detach().numpy()

    def match(self) -> Tuple[int, int]:
        self.load_gem_model()
        des_list = self.get_descriptor_list()
        nlist = 1
        k = 22
        d = 256
        quantizer = faiss.IndexFlatL2(d)
        index = faiss.IndexIVFFlat(quantizer, d, nlist, faiss.METRIC_L2)
        assert not index.is_trained
        index.train(des_list)
        assert index.is_trained
        index.add(des_list)
        eval_seq_descriptors = self.create_value_seq(EVAL_PATH)
        D, I = index.search(eval_seq_descriptors.reshape(1, -1), k)
        return D, I

    def load_gem_model(self) -> None:
        checkpoint = torch.load(self.gem_weights.as_posix())
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.eval()

    def get_descriptor_list(self) -> np.ndarray:
        descriptors_list = np.zeros((int(self.descriptors.shape[0]), 256))
        for index, timestamp in enumerate(self.descriptors_db_path.glob("*.npy")):
            current_batch = read_descriptors(
                index, descriptors=self.descriptors, seq_len=self.seqlen
            )
            self.model.eval()
            current_batch_des = self.model(current_batch)
            current_batch_des = current_batch_des.squeeze(1)
            descriptors_list[int(index), :] = (
                current_batch_des[0, :].cpu().detach().numpy()
            )
        return descriptors_list.astype("float32")
