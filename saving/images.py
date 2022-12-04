#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology


"""Extract images from a rosbag.
"""

import imp
import pdb
from tkinter import image_names
import piexif
import argparse
import os
import pdb

import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
from gps import set_gps_location

DEBAYER_MAP = {"BG": cv2.COLOR_BayerBG2RGB, "GB": cv2.COLOR_BayerGB2RGB}


def parse_args():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_files", help="Input ROS bag or quoted wildcard pattern.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument(
        "--start-index",
        help="Which file to start from if using a wildcard pattern",
        default=0,
        type=int,
    )
    parser.add_argument(
        "--end-index", help="Which file to end on if using a wildcard pattern", type=int
    )
    parser.add_argument(
        "--delta", type=float, help="time between consequetive images", default=0.1
    )
    parser.add_argument(
        "--flip-channels",
        action="store_true",
        help="Flip from RGB to BRG or vice versa",
    )
    parser.add_argument(
        "--debayer",
        action="store_true",
        help="Assume that input is stored as a Bayered image",
    )
    parser.add_argument("--debayer-mode", choices=DEBAYER_MAP.keys(), default="GB", help="GB is for the new payload, BG is for the old payload")
    parser.add_argument("--video-file", help="Save results to a video file")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print bagfile names",
    )
    parser.add_argument("--extension", default=".png", help="Saving image extension")

    args = parser.parse_args()
    return args


def create_video_writer(file_path, FPS=30, wh=(1384, 1032)):
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    writer = cv2.VideoWriter(file_path, fourcc, FPS, wh)
    return writer


def save_images_from_bag(
    bag_file,
    image_topic,
    output_dir,
    flip,
    delta=0.1,
    debayer=False,
    last_time=0,
    debayer_mode="GB",
    video_writer=None,
    GPS_topic="/dji_sdk/gps_position",
    extension=".png"
):
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()

    last_gps = None

    for topic, msg, t in bag.read_messages(topics=[image_topic, GPS_topic]):
        if topic == GPS_topic:
            last_gps = msg
            continue

        if (t.to_time() - last_time) < delta:
            continue

        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if debayer:
            img = cv2.cvtColor(img, DEBAYER_MAP[debayer_mode])

        if flip:
            img = np.flip(img, axis=2)
        image_name = os.path.join(output_dir, "time_%07f" % t.to_time() + extension)
        cv2.imwrite(image_name, img)

        if last_gps is not None and extension==".jpg":
            set_gps_location(
                image_name, last_gps.latitude, last_gps.longitude, last_gps.altitude
            )

        if video_writer is not None:
            video_writer.write(img)
        last_time = t.to_time()

    bag.close()
    return last_time


def main():
    """Extract a folder of images from a rosbag.
    """
    args = parse_args()

    print(
        "Extract images from %s on topic %s into %s"
        % (args.bag_files, args.image_topic, args.output_dir)
    )
    files = sorted(glob(args.bag_files))
    if args.dry_run:
        print(files[args.start_index : args.end_index])
        return

    output_dir = os.path.join(
        args.output_dir, args.image_topic[1:-10].replace("/", "_")
    )

    print(output_dir)
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    if args.video_file is not None:
        print(args.video_file)
        video_writer = create_video_writer(args.video_file)
        import pdb

        pdb.set_trace()
    else:
        video_writer = None

    last_time = 0
    for file in files[args.start_index : args.end_index]:
        print("processing {}".format(file))
        last_time = save_images_from_bag(
            file,
            args.image_topic,
            output_dir,
            args.flip_channels,
            args.delta,
            debayer=args.debayer,
            last_time=last_time,
            debayer_mode=args.debayer_mode,
            video_writer=video_writer,
            extension=args.extension
        )

    if video_writer is not None:
        video_writer.close()


if __name__ == "__main__":
    main()
