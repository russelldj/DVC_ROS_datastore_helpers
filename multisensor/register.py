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


def main(camera_file):
    # Load the xml file
    tree = ET.parse(camera_file)
    root = tree.getroot()
    # Iterate through the camera poses
    for camera in root[0][2]:
        # Print the filename
        print camera.attrib["label"]
        # Get the transform as a (16,) vector
        transform = camera[0].text
        transform = np.fromstring(transform, sep=" ")
        transform = transform.reshape((4, 4))
        print (transform)


if __name__ == "__main__":
    args = parse_args()
    main(args.camera_file)
