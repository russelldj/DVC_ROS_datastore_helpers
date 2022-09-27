import argparse

import rosbag
from cv_bridge import CvBridge
import numpy as np
import pdb
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera-file", default="data/left_image_cameras.xml",
    )
    parser.add_argument("--bagfile")
    parser.add_argument("--topic", default="/left/mapping/image_raw")
    args = parser.parse_args()
    return args


def parse_transforms(camera_file):
    # Load the xml file
    tree = ET.parse(camera_file)
    root = tree.getroot()
    # Iterate through the camera poses

    labels = []
    transforms = []
    for camera in root[0][2]:
        # Print the filename
        # Get the transform as a (16,) vector
        transform = camera[0].text
        transform = np.fromstring(transform, sep=" ")
        transform = transform.reshape((4, 4))
        labels.append(camera.attrib["label"])
        transforms.append(transform)
    return labels, transforms


def project(transform, points=np.array([[0, 0, 0]]), scale=1.1339053033529039e01):
    """
    Arguments
        transform: (4,4)
            Camera to world
        points: (n, 3)
            Points to project
        scale: scalar
            multiplier on input scale
    """
    points = points / scale
    ones = np.ones((points.shape[0], 1))
    points = np.concatenate((points, ones), axis=1)
    return np.dot(transform, points.T).T[:, :3] * scale


def sample_points():
    """
    Create example data
    """
    xs = np.linspace(-30, 30, 21)
    ys = np.linspace(-10, 10, 21)
    xs, ys = np.meshgrid(xs, ys)
    xs, ys = [grid.flatten() for grid in (xs, ys)]
    zs = np.ones_like(xs) * 30
    xyzs = np.vstack((xs, ys, zs)).T
    return xyzs


def main(camera_file):
    labels, transforms = parse_transforms(camera_file)

    centers = []
    points_in_front = []
    lidar_points = sample_points()

    for transform in transforms:
        centers.append(project(transform))
        points_in_front.append(project(transform, points=lidar_points))
    centers = np.vstack(centers)
    points_in_front = np.vstack(points_in_front)
    np.save("data/centers.npy", centers)
    np.save("data/points_in_front.npy", points_in_front)


if __name__ == "__main__":
    args = parse_args()
    main(args.camera_file)
