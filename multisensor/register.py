#!/usr/bin/env python
from bdb import set_trace
from turtle import color, width
import rospy
from sensor_msgs.msg import PointCloud2, Image
from project import texture_lidar


import argparse
import os
import rosbag
from cv_bridge import CvBridge
import cv2
import ros_numpy
import numpy as np
import pdb
from glob import glob
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser("Rosnode for projection")
    parser.add_argument(
        "--camera-file", default="data/left_image_cameras.xml",
    )
    parser.add_argument("--topic", default="/left/mapping/image_raw")
    parser.add_argument("--output-dir", default="/data/global_cloud")
    args = parser.parse_args()
    return args


def parse_transforms(camera_file):
    # Load the xml file
    tree = ET.parse(camera_file)
    root = tree.getroot()
    # Iterate through the camera poses

    labels = []
    transforms = []

    scale = float(root[0][1][0][0][2].text)
    for camera in root[0][2]:
        # Print the filename
        # Get the transform as a (16,) vector
        transform = camera[0].text
        transform = np.fromstring(transform, sep=" ")
        transform = transform.reshape((4, 4))
        # Fix the fact that the transforms are reported in a local scale
        transform[:3, 3] = transform[:3, 3] * scale
        labels.append(camera.attrib["label"])
        transforms.append(transform)
    f = float(root[0][0][0][4][1].text)
    cx = float(root[0][0][0][4][2].text)
    cy = float(root[0][0][0][4][3].text)

    width = float(root[0][0][0][4][0].get("width"))
    height = float(root[0][0][0][4][0].get("height"))

    return labels, np.array(transforms), f, cx, cy, scale, width, height


def project(transform, points=np.array([[0, 0, 0]])):
    """
    Arguments
        transform: (4,4)
            Camera to world
        points: (n, 3)
            Points to project
        scale: scalar
            multiplier on input scale
    """
    ones = np.ones((points.shape[0], 1))
    points = np.concatenate((points, ones), axis=1)
    return np.dot(transform, points.T).T[:, :3]


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
        return_valid=False,
    ):
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.output_dir = output_dir

        self.lidar_topic = lidar_topic
        self.left_cam_topic = left_cam_topic
        self.right_cam_topic = right_cam_topic
        self.spectral_cam_topic = spectral_cam_topic

        self.return_valid = return_valid

        self.left_image_since_lidar = False
        self.right_image_since_lidar = False
        self.spectral_image_since_lidar = False
        self.current_lidar = None

        self.bridge = CvBridge()

        T_cam_imu = np.array(
            [
                [
                    -0.99998569330517,
                    0.0049197739095669015,
                    -0.00209976419080755,
                    0.1182083514751753,
                ],
                [
                    -0.002134472512626063,
                    -0.007060263954729037,
                    0.9999727979800161,
                    0.0553722126932697,
                ],
                [
                    0.004904815192348908,
                    0.9999629735633055,
                    0.0070706640678654875,
                    -0.1544434391424575,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        imu_to_lidar = np.array(
            [
                [0.0580015, -0.998292, 0.00692635, 0],
                [0.998316, 0.0579958, -0.00101483, 0],
                [0.000611399, 0.00697355, 0.999975, 0],
                [0, 0, 0, 1],
            ]
        )
        static = np.dot(T_cam_imu, imu_to_lidar)
        self.static_transform = static
        # self.static_transform = np.array(
        #    [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
        # )

        self.interp_threshold = 0.3

        self.image_width = None
        self.image_height = None

        self.left_image = None
        self.right_image = None
        self.spectral_image = None

        self.f = None
        self.cx = None
        self.cy = None
        self.scale = None
        self.intrinsics = None

    def setup_transforms(self, camera_file):
        (
            image_names,
            self.transforms,
            self.f,
            self.cx,
            self.cy,
            self.scale,
            self.image_width,
            self.image_height,
        ) = parse_transforms(camera_file)

        self.transform_timestamps = np.array([float(x[5:]) for x in image_names])

        # In the agisoft convention, cx, cy are measured from the image center
        self.intrinsics = np.array(
            [
                [self.f, 0, self.cx + self.image_width / 2],
                [0, self.f, self.cy + self.image_height / 2],
                [0, 0, 1],
            ]
        )
        print("Done setting up transforms")

    def get_closest_tranform(self, timestamp):
        diff = np.abs(self.transform_timestamps - timestamp)
        index = np.argmin(diff)
        return self.transforms[index]

    def get_interpolated_transform(self, timestamp, interpolate_rotations=True):
        """
        Take the interpolation between the nearest two transforms

        Note that we average the rotations in matrix space,
        which means that THEY ARE NOT VALID ROTATIONS. 
        However, this approximation is fine for very slow rotations
        """
        diff = np.abs(self.transform_timestamps - timestamp)
        sorted_diff = np.argsort(diff)
        closest_transforms = self.transforms[sorted_diff[:2]]
        closest_diffs = diff[sorted_diff[:2]]
        if np.min(closest_diffs) > self.interp_threshold:
            return None
        weights = closest_diffs / np.sum(closest_diffs)
        # TODO, look at real rotation interpolation
        weighted_transform = np.sum(
            [c * w for c, w in zip(closest_transforms, weights)], axis=0
        )
        if not interpolate_rotations:
            weighted_transform[:3, :3] = closest_transforms[0, :3, :3]

        return weighted_transform

    def lidar_callback(self, data):
        # Get the timestep
        time = data.header.stamp.to_time()
        dynamic_transform = self.get_interpolated_transform(time)
        if dynamic_transform is None:
            print("Invalid interpolation for time " + str(time))
            return
        transform = np.dot(dynamic_transform, self.static_transform)
        self.current_lidar = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

        local_lidar = project(
            transform=self.static_transform, points=self.current_lidar
        )
        colors, valid_inds = texture_lidar(
            local_lidar, self.left_image, self.intrinsics
        )
        if self.left_image is None:
            return
        transformed_lidar = project(transform=transform, points=self.current_lidar)
        colored_lidar = np.concatenate((transformed_lidar, colors), axis=1)
        # Only return the points with color
        if self.return_valid:
            colored_lidar = colored_lidar[valid_inds]
        output_filename = os.path.join(self.output_dir, "lidar_" + str(time) + ".npy")
        np.save(output_filename, colored_lidar)

    def left_camera_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_camera_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def spectral_camera_callback(self, msg):
        self.spectral_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

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

    def save_centers(self):
        centers = []

        for transform in self.transforms:
            centers.append(project(transform))

        centers = np.vstack(centers)
        np.save("data/centers.npy", centers)


if __name__ == "__main__":
    args = parse_args()
    projector = Projector(output_dir=args.output_dir)
    projector.setup_transforms(args.camera_file)
    projector.save_centers()
    projector.listen()

