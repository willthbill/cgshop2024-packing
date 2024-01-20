import fnmatch
import json
import sys
import time
from py.algorithm import run_algorithm, run_repacking_algorithm
from py.configuration import OutputConfiguration
from py.io import read_instance, read_instances, read_output_metadata, write_output, generate_run_id, get_output_directory_for_instance, StreamTee
from py.outputgrabber import OutputGrabber
from py.polygon import Polygon 
from py.visualize import visualize
import os
import subprocess
import concurrent.futures
from multiprocessing import Pool

# time.sleep(20)


def get_best_output_conf(name):

    def translate(items, translations):
        res = []
        for j in range(len(items)):
            new_item = []
            a = items[j].get_approx_representation()
            for i in range(len(a)):
                new_item.append([a[i][0] + translations[j][0], a[i][1] + translations[j][1]])
            res.append(Polygon(new_item))
        return res

    def find_solution_json_files(directories):
        matches = []
        for directory in directories:
            for root, _dirnames, filenames in os.walk(directory):
                for filename in fnmatch.filter(filenames, 'solution.json'):
                    matches.append(os.path.join(root, filename))
        return matches

    solution_files = find_solution_json_files(["output/runs", "/tmp/runs"])

    best_score = -1
    best_file = None
    best_infile = None
    for file in solution_files:
        metadata = read_output_metadata(file)
        assert metadata["output_filename"] == file
        tname = metadata["name"]
        score = metadata["score"]
        if tname == name and score > best_score:
            best_score = score
            best_file = file
            best_infile = metadata["input_filename"]
            print("[py] Found solution with score", score, "in", file)

    assert best_file is not None

    input_conf = read_instance(best_infile)[2]
    with open(best_file, "r") as f:
        data = json.load(f)
    translations = list(zip(data["x_translations"], data["y_translations"]))
    indices = list(map(int, data["item_indices"]))
    included_items = [input_conf.items[i] for i in indices]
    return OutputConfiguration(
        indices,
        translations,
        translate(included_items, translations),
        input_conf
    )


def get_base_output_dir(run_id, use_tmp=True):
    start = "/tmp" if use_tmp else "output"
    dir = f"{start}/runs/{run_id}"
    return dir

def get_output_dir(run_id, name, filename, use_tmp=True):
    dir = f"{get_base_output_dir(run_id, use_tmp=use_tmp)}/{get_output_directory_for_instance(name, filename)}"
    return dir

def get_environment_variables(env):
    should_write = "SHOULD_WRITE" in env and env["SHOULD_WRITE"] == "1"
    in_production = "PRODUCTION" in env and env["PRODUCTION"] == "1"
    should_visualize = "VISUALIZE" in env and env["VISUALIZE"] == "1"
    should_repack = "REPACK" in env and env["REPACK"] == "1"
    description = ""
    if "DESC" in env: description = env["DESC"]
    if in_production and should_write: assert len(description) > 3
    only_stats = "ONLY_STATS" in env and env["ONLY_STATS"] == "1"
    if only_stats: should_write = False
    run_id = env["RUN_ID"] if "RUN_ID" in env else generate_run_id()
    return should_write, description, only_stats, run_id, should_visualize, in_production, should_repack


def run_sequentially(p):
    instances = p[0]
    env = p[1]

    should_write, description, only_stats, run_id, should_visualize, in_production, should_repack = get_environment_variables(env)

    for name, filename, input_conf in instances:

        dir = get_output_dir(run_id, name, filename, not in_production)

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
            s += f"[py] Max number of placed items (upper bound): {input_conf.get_max_number_of_placed_items()}\n"
            s += f"[py] Is repacking: {should_repack}"
            return s

        print(get_information())

        if only_stats:
            print()
            continue
    
        best_output_conf = None
        if should_repack:
            best_output_conf = get_best_output_conf(name)
        start_time = time.time()
        output_conf = None
        if best_output_conf is None:
            output_conf = run_algorithm(input_conf)
        else:
            output_conf = run_repacking_algorithm(input_conf, best_output_conf)
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

def summarize(input_dir, output_dir, in_production):
    command = f"MATCH_INPUT_PATH={input_dir} MATCH_OUTPUT_PATH={output_dir} ./bin/runpythonfile produce.py"
    if not in_production:
        command = f"DEV=1 {command}"
    subprocess.run(command, shell=True, check=False)

instance_files = os.environ["INSTANCE_FILES"]
print("[py-manager] Instance files:", instance_files)
instances = read_instances(instance_files.split(";"))
in_parallel = "RUN_ID" not in os.environ
if in_parallel:
    if "MAX_THREADS" not in os.environ: os.environ["MAX_THREADS"] = "1"
    max_threads = int(os.environ["MAX_THREADS"])
    assert 1 <= max_threads <= 50

    print(f"[py-manager] RUNNING IN PARALLEL WITH MAX_THREADS={max_threads}")

    should_write, description, only_stats, run_id, should_visualize, in_production, should_repack = get_environment_variables(os.environ)

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
    summarize(instance_files, get_base_output_dir(run_id, not in_production), in_production)
else:
    print("[py] RUNNING IN SEQUENCE")
    run_sequentially((instances, os.environ))
