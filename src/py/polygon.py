class Polygon:

    def __init__(self, points = []):
        self.points = points

    def get_approx_representation(self):
        new_points = []
        for p in self.points:
            new_points.append((
                self._get_number(p[0]),
                self._get_number(p[1])
            ))
        return new_points

    def _get_number(self, val):
        if self._get_number_type(val) == "fraction":
            return val[0] / val[1]
        elif self._get_number_type(val) == "fraction-string":
            idx = val.index('/')
            a = int(val[:idx])
            b = int(val[idx+1:])
            return a/b
        else:
            return val

    def _get_number_type(self, v):
        if type(v) == tuple:
            return "fraction"
        elif type(v) == str:
            return "fraction-string"
        else:
            return "number"

