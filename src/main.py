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


def get_environment_variables(env):
    should_write = "SHOULD_WRITE" in env and env["SHOULD_WRITE"] == "1"
    should_visualize = "VISUALIZE" in env and env["VISUALIZE"] == "1"
    description = ""
    if "DESC" in env: description = env["DESC"]
    if should_write: assert len(description) > 3
    only_stats = "ONLY_STATS" in env and env["ONLY_STATS"] == "1"
    if only_stats: should_write = False
    run_id = env["RUN_ID"] if "RUN_ID" in env else generate_run_id()
    return should_write, description, only_stats, run_id, should_visualize


def run_sequentially(p):
    instances = p[0]
    env = p[1]

    should_write, description, only_stats, run_id, should_visualize = get_environment_variables(env)

    for name, filename, input_conf in instances:

        dir = get_output_dir(run_id, name, filename)

        if should_write:
            if not os.path.exists(dir):
                os.makedirs(dir, exist_ok=False)
            # sys.stdout = StreamTee(f'{dir}/stdout.txt', sys.stdout)
            # sys.stderr = StreamTee(f'{dir}/stderr.txt', sys.stderr)

        def get_information():
            s = f"[py] Running algorithm on instance, {name} ({filename})\n"
            s += f"[py] Description: {description}\n"
            s += f"[py] Number of items: {input_conf.get_number_of_items()}\n"
            s += f"[py] Number of vertices on container: {input_conf.get_number_of_vertices_on_container()}\n"
            s += f"[py] Max value: {input_conf.get_max_value()}\n"
            s += f"[py] Max x/y coord: {input_conf.get_max_xy()}\n"
            s += f"[py] Weak upper bound: {input_conf.get_weak_upper_bound()}\n"
            s += f"[py] Strong upper bound: {input_conf.get_strong_upper_bound()}\n"
            s += f"[py] Max number of placed items (upper bound): {input_conf.get_max_number_of_placed_items()}"
            return s

        print(get_information())

        if only_stats:
            print()
            continue

        start_time = time.time()
        output_conf = run_algorithm(input_conf)
        end_time = time.time()
        time_taken = end_time - start_time

        info = get_information() + "\n"
        info += f"[py] Function execution time: {time_taken} seconds\n"
        info += f"[py] Score: {output_conf.get_score()}\n"
        info += f"[py] Finished on instance, {name} ({filename})\n"

        if should_write:
            info += f"[py] Writing output to {dir}\n"
            print(info)
            if should_visualize: visualize(output_conf, show=False, out_file=f"{dir}/visualization.pdf", preserve_coords=True)
            write_output(dir, name, filename, run_id, output_conf, info, description)
        else:
            print(info)
            if should_visualize: visualize(output_conf, show=True, preserve_coords=True)
        print("===========================\n")

def run_command(p):
    command = p[0]
    env = p[1]
    #write_to = p[2]
    #output_stderr = ""
    print(command)
    try:
        # output_stderr = subprocess.run(command, shell=False, check=False, env=env, stderr=subprocess.PIPE, text=True).stderr
        subprocess.run(command, shell=False, check=False, env=env)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")
    # print(output_stderr, file=sys.stderr)
    # if write_to is not None:
    #     assert not os.path.exists(write_to)
    #     with open(write_to, 'w') as f:
    #         f.write(output_stderr)
    print(f"[py-manager] Finished running {command}")


instance_files = os.environ["INSTANCE_FILES"]
print("[py-manager] Instance files:", instance_files)
instances = read_instances(instance_files.split(";"))
in_parallel = "RUN_ID" not in os.environ
if in_parallel:
    if "MAX_THREADS" not in os.environ: os.environ["MAX_THREADS"] = "1"
    max_threads = int(os.environ["MAX_THREADS"])
    assert 1 <= max_threads <= 50

    print(f"[py-manager] RUNNING IN PARALLEL WITH MAX_THREADS={max_threads}")

    should_write, description, only_stats, run_id, should_visualize = get_environment_variables(os.environ)

    # inputs = []
    # for name, filename, input_conf in instances:
    #     env = os.environ.copy()
    #     env['INSTANCE_FILES'] = filename
    #     env['RUN_ID'] = run_id
    #     del env['MAX_THREADS']
    #     inputs.append((
    #         [(name, filename, input_conf)],
    #         env
    #     ))
    

    # with concurrent.futures.ThreadPoolExecutor(max_workers=max_threads) as executor:
    #     executor.map(run_sequentially, inputs)
    # with Pool(max_threads) as pool:
    #     pool.map(run_sequentially, inputs)
    # pool = Pool(processes=max_threads)
    # pool.map(run_sequentially, inputs)
    # pool.close()
    # pool.join()

    commands = []
    for name, filename, input_conf in instances:
        env = os.environ.copy()
        env['INSTANCE_FILES'] = filename
        env['RUN_ID'] = run_id
        del env['MAX_THREADS']
        env["PYTHONPATH"] = ("./out/build:" + env["PYTHONPATH"]) if "PYTHONPATH" in env else "./out/build"
        dir = get_output_dir(run_id, name, filename)
        if should_write:
            if not os.path.exists(dir):
                os.makedirs(dir, exist_ok=False)
        commands.append((
            # ["./bin/runpythonfile", "main.py"],
            ["python3", "src/main.py"],
            env,
            os.path.join(dir, "mem.txt") if should_write else None
        ))
    # with concurrent.futures.ThreadPoolExecutor(max_workers=max_threads) as executor:
    #    executor.map(run_command, commands)
    with Pool(max_threads) as pool:
        pool.map(run_command, commands)
    print("[py manager] FINISHED ALL INSTANCES")
else:
    print("[py] RUNNING IN SEQUENCE")
    run_sequentially((instances, os.environ))
