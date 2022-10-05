import argparse
import numpy as np
import pyvista as pv
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--centers", default="data/centers.npy")
    parser.add_argument("--transformed_dir", default="data/global_clouds")
    parser.add_argument("--start", default=0, type=int)
    parser.add_argument("--step", default=1, type=int)
    parser.add_argument("--number", default=-1, type=int)
    args = parser.parse_args()
    return args


def main(centers_filename, transformed_dir, step, number):
    centers = np.load(centers_filename)
    files = sorted(Path(transformed_dir).glob("*npy"))[:number:step]
    lidar_points = np.concatenate([np.load(x) for x in files], axis=0)

    gray = np.logical_and.reduce(
        [
            lidar_points[:, 3] == 128,
            lidar_points[:, 4] == 128,
            lidar_points[:, 5] == 128,
        ]
    )
    lidar_points = lidar_points[np.logical_not(gray)]

    center_points = pv.PolyData(centers)
    colors = np.flip(lidar_points[:, 3:], axis=1)
    lidar_points = pv.PolyData(lidar_points[:, :3])

    plotter = pv.Plotter()
    plotter.add_mesh(lidar_points, scalars=colors / 255.0, rgb=True)
    plotter.add_mesh(center_points, color="r")
    plotter.add_axes()
    plotter.show()


if __name__ == "__main__":
    args = parse_args()
    main(args.centers, args.transformed_dir, args.step, args.number)

