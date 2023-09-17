from cpp.main import main_algorithm
from py.polygon import Polygon

def run_algorithm(container, items):
    out = main_algorithm(container, items)
    return [
        Polygon(e) for e in out
    ]

