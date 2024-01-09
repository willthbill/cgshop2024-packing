import glob

from py import io


def analyse_dir(directory, sort=None):
    filenames = glob.glob(directory + "/*.json")
    data = []
    for filename in filenames:
        data.append(analyse_file(filename, sort))
    data.sort()
    for e in data:
        print(e[0])
        print(e[1])
        print()


def analyse_file(filename, sort=None):
    name, filename, conf = io.read_instance(filename, sort, expand=True)
    return len(conf.items), filename


# analyse_dir("input/examples_2023-08-17/examples_01", None)
analyse_dir("input/cg24", None)
