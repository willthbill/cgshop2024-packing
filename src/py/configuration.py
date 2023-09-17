class Configuration:

    def __init__(self, container, items, values=[], quantities=[]):
        self.container = container
        self.items = items
        self.values = values
        self.quantities = quantities

    def get_max_value(self):
        if len(self.values) == 0: return 0
        return max(self.values)

    def get_min_value(self):
        if len(self.values) == 0: return 0
        return min(self.values)

    def get_cpp_items(self):
        return [
            (self.values[i], self.quantities[i], self.items[i].get_approx_representation())
            for i in range(len(self.items))
        ]

