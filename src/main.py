import sys
import time
from py.algorithm import run_algorithm
from py.io import read_instances, write_output, generate_run_id, get_output_directory_for_instance, StreamTee
from py.outputgrabber import OutputGrabber 
from py.visualize import visualize
import os

# time.sleep(20)

instance_files = os.environ["INSTANCE_FILES"]
should_write = "SHOULD_WRITE" in os.environ and os.environ["SHOULD_WRITE"] == "1"
only_stats = "ONLY_STATS" in os.environ and os.environ["ONLY_STATS"] == "1"
if only_stats: should_write = False
run_id = generate_run_id()
print("[py] Instance files:", instance_files)
for name, filename, input_conf in read_instances(instance_files.split(";")):

    dir = f"output/runs/{run_id}/{get_output_directory_for_instance(name, filename)}"
    if should_write:
        if not os.path.exists(dir):
            os.makedirs(dir, exist_ok=False)
        sys.stdout = StreamTee(f'{dir}/stdout.txt', sys.stdout)
        sys.stderr = StreamTee(f'{dir}/stderr.txt', sys.stderr)

    print(f"[py] Running algorithm on instance, {name} ({filename})")
    print(f"[py] Number of items: {input_conf.get_number_of_items()}")
    print(f"[py] Max value: {input_conf.get_max_value()}")
    print(f"[py] Max x/y coord: {input_conf.get_max_xy()}")
    print(f"[py] Weak upper bound: {input_conf.get_weak_upper_bound()}")
    print(f"[py] Strong upper bound: {input_conf.get_strong_upper_bound()}")
    print(f"[py] Max number of placed items (upper bound): {input_conf.get_max_number_of_placed_items()}")
    if only_stats:
        print()
        continue

    start_time = time.time()
    output_conf = run_algorithm(input_conf)
    end_time = time.time()
    time_taken = end_time - start_time

    print(f"[py] Function execution time: {time_taken} seconds")
    print(f"[py] Weak upper bound: {output_conf.input_conf.get_weak_upper_bound()}")
    print(f"[py] Strong upper bound: {output_conf.input_conf.get_strong_upper_bound()}")
    print(f"[py] Max number of placed items (upper bound): {input_conf.get_max_number_of_placed_items()}")
    print(f"[py] Score: {output_conf.get_score()}")
    print(f"[py] Finished on instance, {name} ({filename})")

    if should_write:
        print(f"[py] Writing output to {dir}")
        visualize(output_conf, show=False, out_file=f"{dir}/visualization.pdf", preserve_coords=True)
        write_output(dir, name, filename, run_id, output_conf)
    else:
        visualize(output_conf, show=True, preserve_coords=True)
    print("\n\n\n")


### Single file ###
# instance_file = os.environ["INSTANCE_FILE"]
# print("[py] instance file:", instance_file)
# name, input_conf = read_instance(instance_file)
# print(f"[py] Running algorithm on instance, {name}")
# print(f"[py] Number of items: {len(input_conf.items)}")
# output_conf = run_algorithm(input_conf)
# visualize(output_conf, show=True, preserve_coords=True)
# print(f"Score: {output_conf.get_score()}")