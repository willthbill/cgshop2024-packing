from cpp.main import main_algorithm, repacking_algorithm
from py.configuration import InputConfiguration, OutputConfiguration
from py.polygon import Polygon

def run_algorithm(input_conf : InputConfiguration):
    container = input_conf.container.get_approx_representation()
    items = input_conf.get_cpp_items()
    out = main_algorithm(container, items)
    return OutputConfiguration(
        [e[0] for e in out],
        [e[1] for e in out],
        [Polygon(e[2]) for e in out],
        input_conf
    )


def run_repacking_algorithm(input_conf : InputConfiguration, output_conf: OutputConfiguration):
    container = input_conf.container.get_approx_representation()
    items = input_conf.get_cpp_items()
    translations = output_conf.get_cpp_translations()
    out = repacking_algorithm(container, items, translations)
    return OutputConfiguration(
        [e[0] for e in out],
        [e[1] for e in out],
        [Polygon(e[2]) for e in out],
        input_conf
    )
