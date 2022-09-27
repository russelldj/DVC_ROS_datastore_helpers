import argparse
import numpy as np
import pyvista as pv


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("centers")
    parser.add_argument("points_in_front")
    args = parser.parse_args()
    return args


def main(centers_filename, points_in_front_filename):
    centers = np.load(centers_filename)
    points_in_front = np.load(points_in_front_filename)
    center_points = pv.PolyData(centers)
    front_points = pv.PolyData(points_in_front)

    plotter = pv.Plotter()
    plotter.add_mesh(front_points, color="b")
    plotter.add_mesh(center_points, color="r")
    plotter.show()


if __name__ == "__main__":
    args = parse_args()
    main(args.centers, args.points_in_front)

