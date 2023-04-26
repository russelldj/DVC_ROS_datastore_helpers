#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology


"""Extract images from a rosbag.
"""

import pdb
import argparse
import os
import json

import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np
from glob import glob

DEBAYER_MAP = {"BG": cv2.COLOR_BayerBG2RGB, "GB": cv2.COLOR_BayerGB2RGB}


def parse_args():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument(
        "image_bag_files",
        help="Path to bags with images. Input ROS bag or quoted wildcard pattern.",
    )
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("--GPS-topic", help="GPS topic.", default="/fix")
    parser.add_argument(
        "--GPS-bag-files",
        help="Optional path to bags with GPS, if not in the image bags. Input ROS bag or quoted wildcard pattern.",
    )
    parser.add_argument(
        "--start-time", help="What timestamp to start at (unix time)", type=float,
    )
    parser.add_argument(
        "--end-time", help="What timestamp to end at (unix time)", type=float,
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
        "--rotate-180", action="store_true", help="Rotate the image spatially",
    )
    parser.add_argument(
        "--debayer",
        action="store_true",
        help="Assume that input is stored as a Bayered image",
    )
    parser.add_argument(
        "--debayer-mode",
        choices=DEBAYER_MAP.keys(),
        default="GB",
        help="GB is for the new payload, BG is for the old payload",
    )
    parser.add_argument("--video-file", help="Save results to a video file")
    parser.add_argument(
        "--dry-run", action="store_true", help="Print bagfile names",
    )
    parser.add_argument("--extension", default=".jpg", help="Saving image extension")

    args = parser.parse_args()
    return args


def create_video_writer(file_path, FPS=30, wh=(1384, 1032)):
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    writer = cv2.VideoWriter(file_path, fourcc, FPS, wh)
    return writer


def read_GPS(bag_file, gps_dict, gps_topic):
    bag = rosbag.Bag(bag_file, "r")
    for _, msg, t in bag.read_messages(topics=[gps_topic]):
        time = float(t.to_time())
        gps_dict[time] = {
            "lat": float(msg.latitude),
            "lon": float(msg.longitude),
            "alt": float(msg.altitude),
        }
    return gps_dict


def save_images_from_bag(
    bag_file,
    image_topic,
    output_dir,
    flip,
    rotate=False,
    delta=0.1,
    debayer=False,
    last_time=0,
    start_time=None,
    end_time=None,
    debayer_mode="GB",
    video_writer=None,
    extension=".png",
):
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()

    for _, msg, t in bag.read_messages(topics=[image_topic]):

        t_time = t.to_time()
        if (t_time - last_time) < delta:
            continue
        if (start_time is not None and t_time < start_time) or (
            end_time is not None and t_time > end_time
        ):
            continue

        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if debayer:
            img = cv2.cvtColor(img, DEBAYER_MAP[debayer_mode])

        if flip:
            img = np.flip(img, axis=2)
        if rotate:
            img = np.flip(np.flip(img, axis=0), axis=1)

        image_name = os.path.join(output_dir, "time_%07f" % t.to_time() + extension)
        cv2.imwrite(image_name, img)

        if video_writer is not None:
            video_writer.write(img)
        last_time = t.to_time()

    bag.close()
    return last_time


def main():
    """Extract a folder of images from a rosbag.
    """
    args = parse_args()

    image_bag_files = sorted(glob(args.image_bag_files))

    # Handle GPS files
    if args.GPS_bag_files is not None:
        gps_bag_files = sorted(glob(args.image_bag_files))
    else:
        gps_bag_files = image_bag_files

    if args.dry_run:
        # TODO make this better
        print(image_bag_files, gps_bag_files)
        return

    output_dir = os.path.join(
        args.output_dir, args.image_topic[1:-10].replace("/", "_")
    )

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

    gps_dict = {}
    gps_file = os.path.join(output_dir, "gps_info.json")

    for file in gps_bag_files:
        print("Reading GPS from " + file)
        gps_dict = read_GPS(file, gps_dict=gps_dict, gps_topic=args.GPS_topic)
        # Save incrementally to allow early inspection
        with open(gps_file, "w") as gps_file_h:
            gps_file_h.write(json.dumps(gps_dict))


    for file in image_bag_files:
        print("processing {}".format(file))
        last_time = save_images_from_bag(
            file,
            args.image_topic,
            output_dir,
            args.flip_channels,
            rotate=args.rotate_180,
            delta=args.delta,
            debayer=args.debayer,
            last_time=last_time,
            start_time=args.start_time,
            end_time=args.end_time,
            debayer_mode=args.debayer_mode,
            video_writer=video_writer,
            extension=args.extension,
        )

    if video_writer is not None:
        video_writer.close()


if __name__ == "__main__":
    main()
