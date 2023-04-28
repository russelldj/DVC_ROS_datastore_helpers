import argparse
import cv2
import datetime
import time
import os
import json
from gps_tag_images import set_gps_location
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Extract images from a DJI video.")
    parser.add_argument("video_file", help="Path to video file.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("--delta", default=0.5, help="step between images", type=float)
    parser.add_argument("--start-time", type=float)
    parser.add_argument("--stop-time", type=float)
    args = parser.parse_args()
    return args

def parse_subtitles(video_file):
    subtitle_file = video_file.replace(".MP4", ".SRT")
    with open(subtitle_file, "r") as f:
        lines = f.readlines()
    n_frames = len(lines) // 7
    timestamp_inds = [7 * i + 3 for i in range(n_frames)]
    info_inds = [7 * i + 4 for i in range(n_frames)]
    
    timestamp_strings = [lines[i] for i in timestamp_inds]
    info_strings = [lines[i] for i in info_inds]

    timestamps = [datetime.datetime.strptime(x[:-9],"%Y-%m-%d %H:%M:%S") for x in timestamp_strings]
    unix_time_seconds = [time.mktime(x.timetuple()) for x in timestamps]
    microseconds = [(1000 * int(x[-8:-5]) + int(x[-4:-1])) for x in timestamp_strings]
    unix_time_microseconds = [x + y/1e6 for x, y in zip(unix_time_seconds, microseconds)]
    
    info_strings = [x.split(" ") for x in info_strings]
    positions = [{"lat": float(x[-6][:-1]), "lon": float(x[-4][:-1]), "alt": float(x[-2][:-1])} for x in info_strings]

    time_pos_dict = {x: y for x, y in zip(unix_time_microseconds, positions)}
    return time_pos_dict


def main(video_file, output_dir, delta=0.5, start_time=None, stop_time=None):

    video_stem = Path(video_file).stem
    output_dir = str(Path(output_dir, video_stem))
    # Find the subtitles file
    # Open the video
    # Iterate through the video and save each frame

    gps_dict = parse_subtitles(video_file)

    os.makedirs(output_dir, exist_ok=True)
    gps_file = output_dir + "/gps.json"
    with open(gps_file, "w") as gps_file_h:
        gps_file_h.write(json.dumps(gps_dict))

    last_time = 0

    cap = cv2.VideoCapture(video_file)

    for time, pos in gps_dict.items():
        delta_time = time - last_time

        ret, frame = cap.read()
        if not ret:
            return

        if delta_time < delta:
            continue

        if start_time is not None and time < start_time:
            continue

        if stop_time is not None and time < stop_time:
            return 
         
        
        filename = f"{output_dir}/time_{time:06f}.jpg"
        cv2.imwrite(filename, frame)
        set_gps_location(filename, lat=pos["lat"], lng=pos["lon"], altitude=pos["alt"])
        last_time = time

if __name__ == "__main__":
    args = parse_args()
    main(args.video_file, args.output_dir, delta=args.delta, start_time=args.start_time, stop_time=args.stop_time)