import argparse

import rosbag
from cv_bridge import CvBridge
import numpy as np
import pdb
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera-file",
        default="/media/frc-ag-1/Elements/data/ISU/data/site_mungbean_2/08_04_22/collect_2/processed_2/left_image_cameras.xml",
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


def project(transform, point=np.array([0, 0, 0]), scale=1.1339053033529039e01):
    point = point / scale
    point = np.concatenate((point, [1]))
    return np.dot(transform, point)[:3] * scale


def main(camera_file):
    labels, transforms = parse_transforms(camera_file)

    centers = []
    points_in_front = []
    for transform in transforms:
        centers.append(project(transform))
        points_in_front.append(project(transform, point=np.array([0, 0, 1])))
    centers = np.vstack(centers)
    points_in_front = np.vstack(points_in_front)
    np.save("data/centers.npy", centers)
    np.save("data/points_in_front.npy", points_in_front)


if __name__ == "__main__":
    args = parse_args()
    main(args.camera_file)
