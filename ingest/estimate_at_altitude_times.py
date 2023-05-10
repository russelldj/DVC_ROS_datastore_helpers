import yaml
import json
import numpy as np
import matplotlib.pyplot as plt

import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gps-file", required=True)
    parser.add_argument("--info-file")
    parser.add_argument("--vis", action="store_true")
    args = parser.parse_args()
    return args


def main(gps_file, info_file, vis=False, bin_size=0.5):
    """_summary_

    Args:
        gps_file (_type_): _description_
        info_file (_type_): _description_
        vis (bool, optional): _description_. Defaults to False.
        bin_size (int, optional): _description_. Defaults to 1.
    """
    with open(gps_file, "r") as gps_file_h:
        data = json.load(gps_file_h)
    data_array = np.array(
        [[k, v["lat"], v["lon"], v["alt"]] for k, v in data.items()]
    ).astype(float)
    if vis:
        import pyvista as pv

        vis_data = data_array[:, 1:].copy()
        # Address approximate GPS to feet scaling
        vis_data[:, :2] = vis_data[:, :2] * 111139
        height_above_takeoff = data_array[:, 0] - np.min(data_array[:, 0])
        pv.plot(vis_data, scalars=height_above_takeoff)

    altitudes = data_array[:, 3]

    range_of_values = np.max(altitudes) - np.min(altitudes)
    n_bins = int(range_of_values / bin_size)
    hist, bin_edges = np.histogram(altitudes, bins=n_bins)
    # The center of the bin with the most values
    most_common_altitude = bin_edges[np.argmax(hist)] + bin_size / 2
    diffs_to_most_common = np.abs(altitudes - most_common_altitude)

    within_range = diffs_to_most_common < bin_size * 1.5

    if vis:
        pv.plot(vis_data[within_range], scalars=height_above_takeoff[within_range])


if __name__ == "__main__":
    args = parse_args()

    main(args.gps_file, info_file=args.info_file, vis=args.vis)
