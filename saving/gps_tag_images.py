import piexif
import numpy as np
import argparse
from fractions import Fraction
from glob import glob
import json


def to_deg(value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg = int(abs_value)
    t1 = (abs_value - deg) * 60
    min = int(t1)
    sec = round((t1 - min) * 60, 5)
    return (deg, min, sec, loc_value)


def change_to_rational(number):
    """convert a number to rantional
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return (f.numerator, f.denominator)


def set_gps_location(file_name, lat, lng, altitude):
    """Adds GPS position as EXIF metadata
    Keyword arguments:
    file_name -- image file
    lat -- latitude (as float)
    lng -- longitude (as float)
    altitude -- altitude (as float)
    """
    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exiv_lat = (
        change_to_rational(lat_deg[0]),
        change_to_rational(lat_deg[1]),
        change_to_rational(lat_deg[2]),
    )
    exiv_lng = (
        change_to_rational(lng_deg[0]),
        change_to_rational(lng_deg[1]),
        change_to_rational(lng_deg[2]),
    )

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSAltitudeRef: 0,  # Above sea level
        piexif.GPSIFD.GPSAltitude: change_to_rational(round(altitude)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exiv_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exiv_lng,
    }

    exif_dict = {"GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, file_name)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Tag images on disk with GPS from a rosbag."
    )
    parser.add_argument("image_files", help="A wildcard pattern for the images.")
    parser.add_argument(
        "GPS_file", help="Json with GPS information.",
    )
    args = parser.parse_args()
    return args

def main(image_files, gps_file):

    image_files = sorted(glob(image_files))

    with open(gps_file, "r") as gps_file_h:
        gps_dict = json.load(gps_file_h)

    gps_dict = {float(k): v for k, v in gps_dict.items()}
    sorted_timestamps = np.array(list(sorted(list(gps_dict.keys()))))
    min_diffs = []

    for image_file in image_files:
        image_timestamp = float(image_file.split("/")[-1][5:-4])
        diffs = np.abs(sorted_timestamps - image_timestamp)
        min_ind = np.argmin(diffs)
        min_diff = diffs[min_ind]
        min_diffs.append(min_diff)
        min_diff_timestamp = sorted_timestamps[min_ind]
        gps_info = gps_dict[min_diff_timestamp]
        # find the closest timestamp in the dict
        # Tag the data
        set_gps_location(
            image_file,
            lat=gps_info["lat"],
            lng=gps_info["lon"],
            altitude=gps_info["alt"],
        )
    print(f"mean time difference {np.mean(min_diffs)}, max time difference {np.max(min_diffs)}")


if __name__ == "__main__":
    args = parse_args()
    main(args.image_files, gps_file=args.GPS_file)
