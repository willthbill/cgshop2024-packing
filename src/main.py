import sys
print(sys.path)

import time
from py.algorithm import run_algorithm
from py.io import read_instance
from py.visualize import visualize
import os

# time.sleep(20)

instance_file = os.environ["INSTANCE_FILE"]
print("[py] instance file:", instance_file)
name, input_conf = read_instance(instance_file)
print(f"[py] Running algorithm on instance, {name}")
print(f"[py] Number of items: {len(input_conf.items)}")
output_conf = run_algorithm(input_conf)
visualize(output_conf, show=True, preserve_coords=True)
print(f"Score: {output_conf.get_score()}")

