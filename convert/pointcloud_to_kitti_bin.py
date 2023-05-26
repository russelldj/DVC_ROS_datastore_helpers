import argparse
from pypcd import PointCloud
import pdb
import numpy as np
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-cloud",
        default="/home/frc-ag-1/data/Safeforest_CMU_data_dvc/data/site_Gascola/04_27_23/collect_05/processed_03/liosam/transformed/point_cloud_slam.pcd",
    )
    parser.add_argument(
        "--output-file",
        default="/home/frc-ag-1/data/Safeforest_CMU_data_dvc/data/site_Gascola/04_27_23/collect_05/processed_03/kitti_style/velodyne/00000.bin",
    )
    args = parser.parse_args()
    return args


def main(input_cloud, output_file):
    pc = PointCloud.from_path(input_cloud)
    x = pc.pc_data["x"]
    y = pc.pc_data["y"]
    z = pc.pc_data["z"]
    intensity = pc.pc_data["intensity"]
    arr = np.zeros(x.shape[0] * 4, dtype=np.float32)
    arr[::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity
    pdb.set_trace()
    arr.astype("float32").tofile(output_file)


if __name__ == "__main__":

    args = parse_args()

    main(input_cloud=args.input_cloud, output_file=args.output_file)
