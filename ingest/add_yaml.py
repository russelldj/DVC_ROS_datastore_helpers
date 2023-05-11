import os
import argparse
from pathlib import Path
import shutil
import yaml
import collections


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder")
    parser.add_argument("--template-yaml-file")
    args = parser.parse_args()
    return args


def main(
    folder,
    template_yaml_file,
    output_name="collect_info.yaml",
    add_to_git=True,
    overwrite=False,
):
    all_collects = list(Path(folder).rglob("collect_[0-9][0-9]"))

    is_new_files = []

    for collect in all_collects:
        output_yaml_file = Path(collect, output_name)

        if overwrite or not os.path.isfile(output_yaml_file):
            shutil.copyfile(template_yaml_file, output_yaml_file)
            is_new_files.append(True)
        else:  # merge with existing data
            with open(output_yaml_file, "r") as output_yaml_file_h:
                existing_yaml_data = yaml.safe_load(output_yaml_file_h)

            with open(template_yaml_file, "r") as template_yaml_file_h:
                template_yaml_data = yaml.safe_load(template_yaml_file_h)

            if existing_yaml_data is None:
                existing_yaml_data = template_yaml_data
            else:
                # Assume both yaml datas are doubly-nested dicts
                # Add any keys from the template that aren't already present
                for k, v in template_yaml_data.items():
                    if k not in existing_yaml_data:
                        existing_yaml_data[k] = v
                    else:
                        # Assume that this is a nested dict
                        for sub_k, sub_v in v.items():
                            if sub_k not in existing_yaml_data[k]:
                                existing_yaml_data[k][sub_k] = sub_v

            with open(output_yaml_file, "w") as output_yaml_file_h:
                yaml.dump(existing_yaml_data, output_yaml_file_h)

    if add_to_git:
        os.chdir(folder)
        for new_file, collect in zip(is_new_files, all_collects):
            if new_file:
                output_yaml_file = Path(collect, output_name)
                os.system(f"git add {output_yaml_file}")


if __name__ == "__main__":
    args = parse_args()
    main(folder=args.folder, template_yaml_file=args.template_yaml_file)
