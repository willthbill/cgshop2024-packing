import json
from py import math
from py.configuration import Configuration
from py.polygon import Polygon

def to_point_list(json_polygon):
    p = []
    X = json_polygon["x"]
    Y = json_polygon["y"]
    for x,y in zip(X,Y):
        p.append((x,y))
    return p

# Returns a tuple (name, container, items) where
# * name is a string,
# * container is a list of points defining the polygon,
# * items is a list of tuples (value, polygon) that should be packed
def read_instance(filename, sort=None, expand=False):
    with open(filename, 'r') as f:
        json_dict = json.load(f)
        f.close()
    container = Polygon(to_point_list(json_dict["container"]))
    items = [(item['value'], item['quantity'], Polygon(to_point_list(item))) for item in json_dict["items"]]
    if expand:
        items = [(value, 1, polygon) * quantity for (value, quantity, polygon) in items]
    if sort == "value":
        items = sorted(items, key=lambda item: item[0])
    elif sort == "value per area":
        items = sorted(items, key=lambda item: float(item[0])/math.area(item[2]))
    else:
        assert(sort is None)
        pass
    return json_dict["instance_name"], Configuration(
        container,
        [item[2] for item in items],
        [item[0] for item in items],
        [item[1] for item in items],
    )

