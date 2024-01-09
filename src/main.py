import sys
import time
from py.algorithm import run_algorithm
from py.io import read_instances, write_output, generate_run_id, get_output_directory_for_instance, StreamTee
from py.outputgrabber import OutputGrabber 
from py.visualize import visualize
import os
import subprocess
import concurrent.futures
from multiprocessing import Pool

# time.sleep(20)


def get_output_dir(run_id, name, filename):
    dir = f"output/runs/{run_id}/{get_output_directory_for_instance(name, filename)}"
    return dir


def get_environment_variables():
    should_write = "SHOULD_WRITE" in os.environ and os.environ["SHOULD_WRITE"] == "1"
    description = ""
    if "DESC" in os.environ: description = os.environ["DESC"]
    if should_write: assert len(description) > 3
    only_stats = "ONLY_STATS" in os.environ and os.environ["ONLY_STATS"] == "1"
    if only_stats: should_write = False
    run_id = os.environ["RUN_ID"] if "RUN_ID" in os.environ else generate_run_id()
    return should_write, description, only_stats, run_id


def run_sequentially(instances):

    should_write, description, only_stats, run_id = get_environment_variables()

    print("[py] Instance files:", instance_files)
    for name, filename, input_conf in instances:

        dir = get_output_dir(run_id, name, filename)

        if should_write:
            if not os.path.exists(dir):
                os.makedirs(dir, exist_ok=False)
            sys.stdout = StreamTee(f'{dir}/stdout.txt', sys.stdout)
            sys.stderr = StreamTee(f'{dir}/stderr.txt', sys.stderr)

        def print_information():
            print(f"[py] Running algorithm on instance, {name} ({filename})")
            print(f"[py] Description: {description}")
            print(f"[py] Number of items: {input_conf.get_number_of_items()}")
            print(f"[py] Number of vertices on container: {input_conf.get_number_of_vertices_on_container()}")
            print(f"[py] Max value: {input_conf.get_max_value()}")
            print(f"[py] Max x/y coord: {input_conf.get_max_xy()}")
            print(f"[py] Weak upper bound: {input_conf.get_weak_upper_bound()}")
            print(f"[py] Strong upper bound: {input_conf.get_strong_upper_bound()}")
            print(f"[py] Max number of placed items (upper bound): {input_conf.get_max_number_of_placed_items()}")

        print_information()

        if only_stats:
            print()
            continue

        start_time = time.time()
        output_conf = run_algorithm(input_conf)
        end_time = time.time()
        time_taken = end_time - start_time

        print_information()
        print(f"[py] Function execution time: {time_taken} seconds")
        print(f"[py] Score: {output_conf.get_score()}")
        print(f"[py] Finished on instance, {name} ({filename})")

        if should_write:
            print(f"[py] Writing output to {dir}")
            visualize(output_conf, show=False, out_file=f"{dir}/visualization.pdf", preserve_coords=True)
            write_output(dir, name, filename, run_id, output_conf)
        else:
            # visualize(output_conf, show=True, preserve_coords=True)
            pass
        print("===========================\n")


def run_command(p):
    command = p[0]
    env = p[1]
    write_to = p[2]
    output_stderr = ""
    try:
        # output_stderr = subprocess.run(command, shell=True, check=False, env=env, stderr=subprocess.PIPE, text=True).stderr
        subprocess.run(command, shell=True, check=False, env=env)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")
    # print(output_stderr, file=sys.stderr)
    # if write_to is not None:
    #     assert not os.path.exists(write_to)
    #     with open(write_to, 'w') as f:
    #         f.write(output_stderr)
    print(f"Finished running {command}")


instance_files = os.environ["INSTANCE_FILES"]
instances = read_instances(instance_files.split(";"))
in_parallel = "RUN_ID" not in os.environ
if in_parallel:
    if "MAX_THREADS" not in os.environ: os.environ["MAX_THREADS"] = "1"
    max_threads = int(os.environ["MAX_THREADS"])
    assert 1 <= max_threads <= 50

    print(f"[py-manager] RUNNING IN PARALLEL WITH MAX_THREADS={max_threads}")

    should_write, description, only_stats, run_id = get_environment_variables()

    commands = []
    for name, filename, input_conf in instances:
        env = os.environ.copy()
        env['INSTANCE_FILES'] = filename
        env['RUN_ID'] = run_id
        del env['MAX_THREADS']
        dir = get_output_dir(run_id, name, filename)
        if should_write:
            if not os.path.exists(dir):
                os.makedirs(dir, exist_ok=False)
        commands.append((
            "time -v ./bin/runpythonfile main.py",
            env,
            os.path.join(dir, "mem.txt") if should_write else None
        ))
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_threads) as executor:
        executor.map(run_command, commands)
    # with Pool(max_threads) as pool:
    #     pool.map(run_command, commands)
else:
    print("[py-manager] RUNNING IN SEQUENCE")
    run_sequentially(instances)
