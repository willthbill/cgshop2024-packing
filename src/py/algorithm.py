from cpp.main import main_algorithm
from py.configuration import InputConfiguration, OutputConfiguration
from py.polygon import Polygon

def run_algorithm(input_conf : InputConfiguration):
    container = input_conf.container.get_approx_representation()
    items = input_conf.get_cpp_items()
    out = main_algorithm(container, items)
    used_values = [input_conf.values[e[0]] for e in out]
    used_items = [Polygon(e[1]) for e in out]
    return OutputConfiguration(used_items, used_values, input_conf)

