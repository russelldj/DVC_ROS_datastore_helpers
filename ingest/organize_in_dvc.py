import argparse
import os
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir", help="Where the data stored as collect_{03:d} is"
    )
    parser.add_argument(
        "--output-dir",
        help="Where to write to data. Data in the format collect_{03:d} should be there or will be populated",
    )
    parser.add_argument(
        "--data-name",
        choices=("xavier", "nuc"),
        required=True,
        help="What to call the  data within raw",
    )
    parser.add_argument(
        "--copy",
        action="store_true",
        help="copy data from srs to dest rather than moving, which is generally much faster",
    )

    args = parser.parse_args()
    return args


def main(input_dir, output_dir, data_name, copy, repeat_push=5, push_jobs=4):
    # Find all the collects matching the template string
    collects = sorted(Path(input_dir).glob("collect_[0-9][0-9]"))
    collect_folder_names = [list(c.parts)[-1] for c in collects]

    output_folder_names = [
        Path(output_dir, collect_folder_name, "raw", data_name)
        for collect_folder_name in collect_folder_names
    ]
    for input_folder_name, output_folder_name in zip(collects, output_folder_names):
        if copy:
            raise NotImplementedError()
        else:
            os.makedirs(output_folder_name.parent, exist_ok=True)
            os.rename(input_folder_name, output_folder_name)
    os.chdir(output_dir)
    #
    os.system(f"dvc add -R {output_dir}")
    for _ in range(repeat_push):
        os.system(f"dvc push -R {output_dir} -j {push_jobs}")


if __name__ == "__main__":
    args = parse_args()

    main(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        data_name=args.data_name,
        copy=args.copy,
    )
