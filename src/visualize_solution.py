import os, json
from py.configuration import OutputConfiguration
from py.io import read_instance
from py.polygon import Polygon

from py.visualize import visualize

assert "FILENAME" in os.environ, "Please set the environment variable FILENAME to the path of the solution file"
outfile = os.environ["FILENAME"]
assert os.path.exists(outfile), f"File {outfile} does not exist"

with open(outfile, "r") as f:
    data = json.load(f)
infile = data["meta"]["input_filename"]
input_conf = read_instance(infile)[2]


def translate(items, translations):
    res = []
    for j in range(len(items)):
        new_item = []
        a = items[j].get_approx_representation()
        for i in range(len(a)):
            new_item.append([a[i][0] + translations[j][0], a[i][1] + translations[j][1]])
        res.append(Polygon(new_item))
    return res


translations = list(zip(data["x_translations"], data["y_translations"]))
indices = list(map(int, data["item_indices"]))
included_items = [input_conf.items[i] for i in indices]

output_conf = OutputConfiguration(
    indices,
    translations,
    translate(included_items, translations),
    input_conf
)

visualize(output_conf, show=True, preserve_coords=True)
