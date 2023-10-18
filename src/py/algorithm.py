from cpp.main import main_algorithm
from py.configuration import InputConfiguration, OutputConfiguration

def run_algorithm(input_conf : InputConfiguration):
    container = input_conf.container.get_approx_representation()
    items = input_conf.get_cpp_items()
    out = main_algorithm(container, items)
    return OutputConfiguration(
        [e[0] for e in out],
        [e[1] for e in out],
        [e[2] for e in out],
        input_conf
    )

