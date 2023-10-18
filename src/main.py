from py.algorithm import run_algorithm
from py.io import read_instance
from py.visualize import visualize
import os

instance_file = os.environ["INSTANCE_FILE"]
print(instance_file)
name, input_conf = read_instance(instance_file)
print(f"--- Running algorithm on instance, {name}")
print(f"  - number of items: {len(input_conf.items)}")
output_conf = run_algorithm(input_conf)
visualize(output_conf, show=True, preserve_coords=True)
print(f"score: {output_conf.get_score()}")

