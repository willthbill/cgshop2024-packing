from py.algorithm import run_algorithm
from py.configuration import Configuration
from py.io import read_instance
from py.visualize import visualize
import os

instance_file = os.environ["INSTANCE_FILE"]
print(instance_file)
name, conf = read_instance(instance_file)
print(f"--- Running algorithm on instance, {name}")
print(f"  - number of items: {len(conf.items)}")
output = run_algorithm(conf.container.get_approx_representation(), conf.get_cpp_items())
print(output)
visualize(Configuration(conf.container, output), show=True, preserve_coords=True)

