import matplotlib.pyplot as plt

def parse_input():
    old_boundary = []
    new_boundary = []
    current_boundary = None

    while True:
        try:
            line = input()
            if '["Old boundary"]' in line:
                current_boundary = old_boundary
            elif '["New boundary"]' in line:
                current_boundary = new_boundary
            elif '[p]' in line:
                # [p] = [625/1 -917/1]
                parts = line[6:].split(' ')
                x, y = parts[0][1:], parts[1][:-1]
                x = int(x.split('/')[0]) / int(x.split('/')[1])
                y = int(y.split('/')[0]) / int(y.split('/')[1])
                current_boundary.append((x, y))
        except EOFError:
            break

    return old_boundary, new_boundary

def parse_input2():
    polygons = []
    current = None
    while True:
        try:
            line = input()
            if '[p]' not in line:
                if current is not None and len(current): polygons.append(current)
                current = []
            else:
                #[p] = [625/1 -917/1]
                parts = line[6:].split(' ')
                x, y = parts[0][1:], parts[1][:-1]
                x = int(x.split('/')[0]) / int(x.split('/')[1])
                y = int(y.split('/')[0]) / int(y.split('/')[1])
                current.append((x, y))
        except EOFError:
            break
    if current is not None and len(current): polygons.append(current)
    return polygons

def plot_polygons(old_boundary, new_boundary):
    plt.figure(figsize=(10, 8))
    print(len(old_boundary), len(new_boundary))
    
    if old_boundary:
        x, y = zip(*old_boundary + [old_boundary[0]])  # close the polygon
        plt.plot(x, y, label="Old Boundary")

    if new_boundary:
        x, y = zip(*new_boundary + [new_boundary[0]])  # close the polygon
        plt.plot(x, y, label="New Boundary")

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Old vs New Boundary Polygons")
    plt.legend()
    plt.show()

def plot_polygons2(polygons):
    plt.figure(figsize=(10, 8))
    for polygon in polygons:
        print(len(polygon), end=" ")
        print(polygon)
    print()
    
    colors = "rgbybw"
    for i, polygon in enumerate(polygons):
        x, y = zip(*(polygon + [polygon[0]]))  # close the polygon
        print(x,y)
        plt.plot(x, y, colors[i] + '-o', label="polygon_" + str(i))
        plt.axis('equal')

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Old vs New Boundary Polygons")
    plt.legend()
    plt.show()

def main():
    #old_boundary, new_boundary = parse_input()
    #plot_polygons(old_boundary, new_boundary)
    polygons = parse_input2()
    plot_polygons2(polygons)

if __name__ == "__main__":
    main()

