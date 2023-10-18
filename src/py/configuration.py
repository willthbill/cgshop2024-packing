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

class InputConfiguration(Configuration):

    def __init__(self, container, items, values, quantities):
        super().__init__(container, items, values)
        self.quantities = quantities

    def get_cpp_items(self):
        return [
            (self.values[i], self.quantities[i], self.items[i].get_approx_representation())
            for i in range(len(self.items))
        ]

class OutputConfiguration(Configuration):

    def __init__(self, indices, translations, items, input_conf):
        values = [input_conf.values[idx] for idx in indices]
        super().__init__(input_conf.container, items, values)
        self.translations = translations
        self.indices = indices
        self.input_conf = input_conf

    def get_score(self):
        return sum(self.values)

