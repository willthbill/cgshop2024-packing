import os
import fnmatch
import json
import subprocess
from py.io import read_output_metadata


def find_solution_json_files(directory):
    matches = []
    for root, _dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, 'solution.json'):
            matches.append(os.path.join(root, filename))
    return matches


solution_files = find_solution_json_files("output/runs")
print(len(solution_files), "solutions found\n")

runs = {}
for file in solution_files:
    metadata = read_output_metadata(file)
    assert metadata["output_filename"] == file
    id = metadata["run_id"]
    if id not in runs:
        runs[id] = {}

    def add(key, value):
        if key not in runs[id]:
            runs[id][key] = value
        else:
            assert runs[id][key] == value

    if "description" in metadata:
        add("description", metadata["description"])
    add("directory", os.path.dirname(metadata["output_dir"]))
    if "cnt" not in runs[id]:
        add("cnt", 1)
    else:
        runs[id]["cnt"] += 1
    

def get_max_running_time(run_id):
    command = f"cat output/runs/{run_id}/*/stdout.txt | grep 'Function execution time' | grep -oE '[0-9]+(\.[0-9]+)?' | sort -n | tail -n 1"
    res = subprocess.run(command, shell=True, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout
    res = float(res.strip())
    return res

for id in runs:
    if runs[id]["cnt"] != 180:
        continue
    print("==========================")
    print(id)
    print(json.dumps(runs[id], indent=4))
    command = f"MATCH_OUTPUT_PATH={id} ./bin/runpythonfile produce.py | grep Average"
    print("Running:", command)
    subprocess.run(command, shell=True, check=False)
    print("Max running time:", get_max_running_time(id))
    print("==========================\n")
