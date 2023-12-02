import glob

from py import io


def analyse_dir(directory, sort=None):
    filenames = glob.glob(directory + "/*.json")
    for filename in filenames:
        analyse_file(filename, sort)


def analyse_file(filename, sort=None):
    name, conf = io.read_instance(filename, sort, expand=True)
    print("number of items: ", len(conf.items))


analyse_dir("input/examples_2023-08-17/examples_01", "value per area")
