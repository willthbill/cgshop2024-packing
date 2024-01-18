from py import math


class Configuration:

    def __init__(self, container, items, values):
        self.container = container
        self.items = items
        self.values = values

    def get_max_value(self):
        if len(self.values) == 0: return 0
        return max(self.values)

    def get_min_value(self):
        if len(self.values) == 0: return 0
        return min(self.values)
    
    def get_max_x(self):
        mx = 0
        for item in self.items + [self.container]:
            for point in item.points:
                mx = max(mx, point[0])
        return mx

    def get_max_y(self):
        mx = 0
        for item in self.items + [self.container]:
            for point in item.points:
                mx = max(mx, point[1])
        return mx

    def get_max_xy(self):
        return max(self.get_max_x(), self.get_max_y())


class InputConfiguration(Configuration):

    def __init__(self, container, items, values, quantities):
        super().__init__(container, items, values)
        self.quantities = quantities

    def get_number_of_items(self):
        return sum(self.quantities)

    def get_number_of_vertices_on_container(self):
        return len(self.container.points)

    def get_cpp_items(self):
        return [
            (self.values[i], self.quantities[i], self.items[i].get_approx_representation())
            for i in range(len(self.items))
        ]
    
    def get_weak_upper_bound(self):
        res = 0
        for i in range(len(self.values)):
            res += self.values[i] * self.quantities[i]
        return res

    # fractional knapsack
    def get_strong_upper_bound(self):
        a = []
        for idx in range(len(self.items)):
            a.append([
                self.values[idx] / math.area(self.items[idx]),
                math.area(self.items[idx]) * self.quantities[idx]
            ])
        a.sort(reverse=True)
        area = math.area(self.container)
        idx = 0
        res = 0
        while area > 0 and idx < len(a):
            am = min(a[idx][1], area)
            a[idx][1] -= am
            area -= am
            res += a[idx][0] * am
            if a[idx][1] <= 0:
                idx += 1
        return res

    # fractional knapsack
    def get_max_number_of_placed_items(self):
        a = []
        for idx in range(len(self.items)):
            a.append([
                math.area(self.items[idx]),
                self.quantities[idx]
            ])
        a.sort()
        area = math.area(self.container)
        idx = 0
        res = 0
        while area > 0 and idx < len(a):
            if a[idx][0] > area:
                break
            am = min(a[idx][1], area // a[idx][0])
            a[idx][1] -= am
            area -= am * a[idx][0]
            res += am
            if a[idx][1] == 0:
                idx += 1
        return res


class OutputConfiguration(Configuration):

    def __init__(self, indices, translations, items, input_conf):
        values = [input_conf.values[idx] for idx in indices]
        super().__init__(input_conf.container, items, values)
        self.translations = []
        for t in translations:
            a = None
            b = None
            if type(t[0]) is str and "/" in t[0]:
                assert t[0][-2:] == "/1"
                a = int(t[0][:-2])
            else:
                a = int(t[0])
            if type(t[1]) is str and "/" in t[1]:
                assert t[1][-2:] == "/1"
                b = int(t[1][:-2])
            else:
                b = int(t[1])
            self.translations.append((a, b))
        self.indices = indices
        self.input_conf = input_conf

    def get_score(self):
        score = sum(self.values)
        assert self.input_conf.get_weak_upper_bound() >= self.input_conf.get_strong_upper_bound() >= score
        return score

    def get_cpp_translations(self):
        return [
            (self.indices[i], self.translations[i])
            for i in range(len(self.indices))
        ]
