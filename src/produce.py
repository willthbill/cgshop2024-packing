import os
import fnmatch
import zipfile
from py.io import read_output_metadata


def find_solution_json_files(directory):
    matches = []
    for root, _dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, 'solution.json'):
            matches.append(os.path.join(root, filename))
    return matches


solution_files = find_solution_json_files("output")
print(len(solution_files), "solutions found\n")

names = {}
for file in solution_files:
    metadata = read_output_metadata(file)
    assert metadata["output_filename"] == file
    name = metadata["name"]
    if name not in names:
        names[name] = []
    names[name].append((metadata))

best_filenames = []
for name in names:
    names[name].sort(key=lambda x: x["score"], reverse=True)
    scores = [d["score"] for d in names[name]]
    best = names[name][0]
    weak = best["weak_upper_bound"]
    assert all(d["weak_upper_bound"] == weak for d in names[name])
    strong = best["strong_upper_bound"]
    assert all(d["strong_upper_bound"] == strong for d in names[name])
    assert all(s <= strong <= weak for s in scores)
    print(f"{name}")
    print(f"   {weak} >= {strong} >= {scores}")
    print(f"   relative to strong: {scores[0] / strong}")
    print(f"   best: {best['output_dir']}")
    best_filenames.append(best["output_filename"])

submission_filename = "submission.zip"
with zipfile.ZipFile(submission_filename, 'w') as zipf:
    for file in best_filenames:
        arcname = file.replace(os.sep, "SEP")
        zipf.write(file, arcname=arcname)
print(f"\nProduced {submission_filename}")
