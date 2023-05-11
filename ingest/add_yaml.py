import os
import argparse
from pathlib import Path
import shutil


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder")
    parser.add_argument("--yaml-file")
    args = parser.parse_args()
    return args


def main(folder, yaml_file, output_name="collect_info.yaml", add_to_git=True):
    all_collects = list(Path(folder).rglob("collect_[0-9][0-9]"))

    new_files = []
    for collect in all_collects:
        output_path = Path(collect, output_name)
        if not os.path.isfile(output_path):
            shutil.copyfile(yaml_file, output_path)
            new_files.append(True)
        else:
            new_files.append(False)

    if add_to_git:
        os.chdir(folder)
        for new_file, collect in zip(new_files, all_collects):
            if new_file:
                output_path = Path(collect, output_name)
                os.system(f"git add {output_path}")


if __name__ == "__main__":
    args = parse_args()
    main(folder=args.folder, yaml_file=args.yaml_file)
