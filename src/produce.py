import os
import fnmatch
import zipfile
from py.io import read_output_metadata


def find_solution_json_files(directories):
    matches = []
    for directory in directories:
        for root, _dirnames, filenames in os.walk(directory):
            for filename in fnmatch.filter(filenames, 'solution.json'):
                matches.append(os.path.join(root, filename))
    return matches


DEV="DEV" in os.environ and os.environ["DEV"] == "1"
match_input_path=os.environ["MATCH_INPUT_PATH"] if "MATCH_INPUT_PATH" in os.environ else "input/cg24"
match_output_path=os.environ["MATCH_OUTPUT_PATH"] if "MATCH_OUTPUT_PATH" in os.environ else "output/runs"
solution_files = find_solution_json_files(["output/runs"] + (["/tmp/runs"] if DEV else []))
print(len(solution_files), "solutions found\n")

names = {}
for file in solution_files:
    metadata = read_output_metadata(file)
    assert metadata["output_filename"] == file
    name = metadata["name"]
    if name not in names:
        names[name] = []
    names[name].append((metadata))

_names = list(names.keys())
for name in _names:
    if not any(match_input_path in e["input_filename"] for e in names[name]):
        del names[name]

best_filenames = []
average_rel_worst = 0
average_rel_strong = 0
average_rel_best = 0
cnt = 0
for name in names:
    names[name].sort(key=lambda x: x["score"], reverse=True)
    worst = names[name][-1]["score"]
    best_of_all = names[name][0]["score"]
    names[name] = list(filter(lambda x: match_output_path in x["output_filename"], names[name]))
    if len(names[name]) == 0:
        continue
    scores = [d["score"] for d in names[name]]
    best = names[name][0]
    weak = best["weak_upper_bound"]
    assert all(d["weak_upper_bound"] == weak for d in names[name])
    strong = best["strong_upper_bound"]
    assert all(d["strong_upper_bound"] == strong for d in names[name])
    assert all(s <= strong <= weak for s in scores)
    print(f"{name}")
    print(f"   {weak} >= {strong} >= {scores}")
    rel_strong = scores[0] / strong
    rel_worst = scores[0] / worst
    rel_best = scores[0] / best_of_all
    average_rel_strong += rel_strong
    average_rel_worst += rel_worst
    average_rel_best += rel_best
    cnt += 1
    print(f"   relative to strong: {rel_strong}")
    print(f"   relative to worst: {rel_worst}")
    print(f"   relative to best of all: {rel_best}")
    print(f"   best: {best['output_dir']}")
    best_filenames.append(best["output_filename"])

average_rel_strong /= cnt
average_rel_worst /= cnt
average_rel_best /= cnt

print(f"\nAverage relative to strong: {average_rel_strong}")
print(f"Average relative to worst: {average_rel_worst}")
print(f"Average relative to best of all: {average_rel_best}")

submission_filename = "submission.zip"
with zipfile.ZipFile(submission_filename, 'w') as zipf:
    for file in best_filenames:
        arcname = file.replace(os.sep, "SEP")
        zipf.write(file, arcname=arcname)
print(f"\nProduced {submission_filename}")
