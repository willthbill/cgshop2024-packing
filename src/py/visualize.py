import glob
import math
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import matplotlib as mpl
from py import io

mpl.use('TkAgg')

def append_items_arrange(patches, items, container):
    x_start = 1.5*max([p[0] for p in container])

    num_columns = math.floor(math.sqrt(len(items)))
    num_rows = math.ceil(len(items)/num_columns)

    x_step = 1.5*max([p[0] for P in items for p in P])
    y_step = 1.5*max([p[1] for P in items for p in P])

    for j in range(num_rows):
        for i in range(num_columns):
            item_index = j*num_columns + i
            if item_index >= len(items):
                return
            item = items[item_index]
            x_off = x_start + i*x_step
            y_off = j*y_step
            P = [(p[0]+x_off, p[1]+y_off) for p in item]
            patches.append(Polygon(P, closed=True, ec="black"))

def append_items(patches, items):
    for P in items:
        patches.append(Polygon(P, closed=True))

def visualize(conf, show=True, out_file=None, preserve_coords=False):
    fig, ax = plt.subplots()

    container = conf.container.get_approx_representation()
    print("[py] container: ", container)
    items = [item.get_approx_representation() for item in conf.items]
    for item in items: print("[py] item:", item)

    min_value=conf.get_min_value()
    max_value=conf.get_max_value()
    min_value -= (max_value - min_value) * 0.2

    cmap = mpl.cm.hot_r
    norm = mpl.colors.Normalize(vmin=min_value, vmax=max_value)
    
    ax.add_patch(Polygon(container, closed=True, fc="white", ec="black"))

    patches = []
    if preserve_coords: append_items(patches, items)
    else: append_items_arrange(patches, items, container)

    patch_collection = PatchCollection(patches, cmap=cmap, norm=norm, ec="black")#, lw=.002)
    if min_value < max_value:
        patch_collection.set_array(conf.values)
        patch_collection.set_clim([min_value, max_value])
        plt.colorbar(patch_collection,shrink=.8,ax=ax)
    ax.add_collection(patch_collection)

    ax.margins(.1)
    ax.axis("off")
    ax.axis("equal")
    ax.autoscale()
    fig.tight_layout()
    if show: 
        print("rendering")
        plt.show()
    if out_file is not None:
        plt.savefig(out_file, dpi=300)
    plt.close()

def visualize_instances_dir(directory, sort=None):
    filenames = glob.glob(directory + "/*.json")
    for filename in filenames:
        visualize_instance_file(filename, sort)

def visualize_instance_file(filename, sort=None):
    name, conf = io.read_instance(filename, sort, expand=True)
    print(name)
    visualize(conf, show=True)

