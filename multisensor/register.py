#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image


import argparse
import os
import rosbag
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import pdb
from glob import glob
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera-file", default="data/left_image_cameras.xml",
    )
    parser.add_argument(
        "--bag-files",
        default="/media/frc-ag-1/Elements/data/ISU/data/site_mungbean_2/08_04_22/collect_2/raw/*.bag",
        help="Filename or quoted glob string",
    )
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


class Projector:
    def __init__(
        self,
        lidar_topic="/velodyne_points",
        left_cam_topic="/left/camera/image_color",
        right_cam_topic="/right/camera/image_color",
        spectral_cam_topic="/webcam/image_raw",
        output_dir="data/global_clouds",
    ):
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.output_dir = output_dir

        self.lidar_topic = lidar_topic
        self.left_cam_topic = left_cam_topic
        self.right_cam_topic = right_cam_topic
        self.spectral_cam_topic = spectral_cam_topic

        self.left_image_since_lidar = False
        self.right_image_since_lidar = False
        self.spectral_image_since_lidar = False
        self.current_lidar = None

        self.static_transform = np.array(
            [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
        )

    def setup_transforms(self, camera_file):
        image_names, self.transforms = parse_transforms(camera_file)
        self.transform_timestamps = np.array([float(x[5:]) for x in image_names])
        print("Done setting up transforms")

    def get_closest_tranform(self, timestamp):
        diff = np.abs(self.transform_timestamps - timestamp)
        index = np.argmin(diff)
        return self.transforms[index]

    def lidar_callback(self, data):
        # Get the timestep
        time = data.header.stamp.to_time()
        transform = np.dot(self.get_closest_tranform(time), self.static_transform)
        self.current_lidar = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        transformed_lidar = project(transform=transform, points=self.current_lidar)
        output_filename = os.path.join(self.output_dir, "lidar_" + str(time) + ".npy")
        np.save(output_filename, transformed_lidar)

    def left_camera_callback(self, data):
        pass

    def right_camera_callback(self, data):
        pass

    def spectral_camera_callback(self, data):
        pass

    def listen(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node("listener", anonymous=True)

        rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        rospy.Subscriber(self.left_cam_topic, Image, self.left_camera_callback)
        rospy.Subscriber(self.right_cam_topic, Image, self.right_camera_callback)
        rospy.Subscriber(self.spectral_cam_topic, Image, self.spectral_camera_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def example_projection(self):
        centers = []
        points_in_front = []
        lidar_points = sample_points()

        for transform in self.ftransforms:
            centers.append(project(transform))
            points_in_front.append(project(transform, points=lidar_points))

        centers = np.vstack(centers)
        points_in_front = np.vstack(points_in_front)
        np.save("data/centers.npy", centers)
        np.save("data/points_in_front.npy", points_in_front)


if __name__ == "__main__":
    args = parse_args()
    projector = Projector()
    projector.setup_transforms(args.camera_file)
    projector.listen()

