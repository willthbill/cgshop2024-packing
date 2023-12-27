import json
import os
import glob
import time
import hashlib
import sys
from datetime import datetime

from py import math
from py.configuration import InputConfiguration, OutputConfiguration
from py.polygon import Polygon


def to_point_list(json_polygon):
    p = []
    X = json_polygon["x"]
    Y = json_polygon["y"]
    for x,y in zip(X,Y):
        p.append((x,y))
    return p


# Returns a tuple (name, container, items) where
# * name is a string,
# * container is a list of points defining the polygon,
# * items is a list of tuples (value, polygon) that should be packed
def read_instance(filename, sort=None, expand=False):
    with open(filename, 'r') as f:
        json_dict = json.load(f)
        f.close()
    container = Polygon(to_point_list(json_dict["container"]))
    items = [(item['value'], item['quantity'], Polygon(to_point_list(item))) for item in json_dict["items"]]
    if expand:
        items = [(value, 1, polygon) * quantity for (value, quantity, polygon) in items]
    if sort == "value":
        items = sorted(items, key=lambda item: item[0])
    elif sort == "value per area":
        items = sorted(items, key=lambda item: float(item[0])/math.area(item[2]))
    else:
        assert(sort is None)
        pass
    return json_dict["instance_name"], filename, InputConfiguration(
        container,
        [item[2] for item in items],
        [item[0] for item in items],
        [item[1] for item in items],
    )


def read_output_metadata(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    metadata = data["metadata"]
    return metadata


def read_instances(paths, sort=None, expand=False):
    for path in paths:
        # Expand wildcards
        if '*' in path:
            for expanded_path in glob.glob(path):
                yield read_instance(expanded_path, sort=sort, expand=expand)
        # Handle directories
        elif os.path.isdir(path):
            for dirpath, _, filenames in os.walk(path):
                for filename in filenames:
                    full_path = os.path.join(dirpath, filename)
                    yield read_instance(full_path, sort=sort, expand=expand)
        # Handle single files
        else:
            yield read_instance(path, sort=sort, expand=expand)


def generate_time_based_id():
    # Get current time in microseconds
    timestamp = int(time.time() * 1_000_000)

    # Generate cryptographic-quality random data
    random_bytes = os.urandom(8)  # 8 bytes of random data

    # Combine the timestamp and random data
    combined = f"{timestamp}".encode() + random_bytes

    # Use a hash function for a fixed-length ID
    unique_id = hashlib.sha1(combined).hexdigest()[:16]  # 16 characters

    return unique_id


def generate_run_id():
    current_time_str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    return f"{current_time_str}_{generate_time_based_id()[:8]}"


def get_output_directory_for_instance(name, input_filename):
    unique_part = hashlib.sha1(
        f"{name}==={input_filename}==={generate_time_based_id()}".encode()
    ).hexdigest()[:8]
    filename = f"{name}_{unique_part}"
    return filename


def write_output(dir, name, input_filename, run_id, output_conf):
    # Ensure the directory exists
    assert os.path.exists(dir)

    output_filename = os.path.join(dir, 'output.json')
    metadata = {
        'name': name,
        'input_filename': input_filename,
        'output_dir': dir,
        'output_filename': output_filename,
        'run_id': run_id,
        'score': output_conf.get_score(),
        'weak_upper_bound': output_conf.input_conf.get_weak_upper_bound(),
        'strong_upper_bound': output_conf.input_conf.get_strong_upper_bound(),
    }

    with open(output_filename, 'w') as f:
        output = {
            "type": "cgshop2024_solution",
            "instance_name": name,
            "num_included_items": len(output_conf.indices),
            "meta": metadata,
            "item_indices": output_conf.indices,
            "x_translations": [e[0] for e in output_conf.translations],
            "y_translations": [e[1] for e in output_conf.translations],
        }
        json.dump(output, f, indent=4)

    with open(os.path.join(dir, "metadata.json"), 'w') as f:
        json.dump(metadata, f, indent=4)

    with open(os.path.join(dir, "env.txt"), 'w') as f:
        for key, value in os.environ.items():
            f.write(f'{key}: {value}\n')


class StreamTee(object):
    def __init__(self, file_name, stream):
        self.file = open(file_name, 'a')  # Append mode
        self.stream = stream

    def write(self, data):
        self.file.write(data)
        self.stream.write(data)

    def flush(self):
        self.file.flush()
        self.stream.flush()

    def close(self):
        self.file.close()
