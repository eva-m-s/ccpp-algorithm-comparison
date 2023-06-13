import random
import numpy as np


class ProblemInstance:
    number_of_controllers: int = 3
    number_of_switches: int
    switches: list
    switches_loads: list
    locations: list
    x_range: tuple = (0, 90)
    y_range: tuple = (0, 180)
    distances: list
    capacities_model: list
    capacities_algorithm: list

    def __init__(self, number_of_switches: int, loads: list) -> None:
        if len(loads) != number_of_switches:
            raise ValueError("Number of switches does not match with number of passed loads")

        self.number_of_switches = number_of_switches
        self.switches = list(range(number_of_switches))
        self.locations = self.initialize_locations(number_of_switches)
        self.distances = self.initialize_distances()
        self.switches_loads = loads

    def initialize_locations(self, number_of_switches: int) -> list:
        locations = []
        for switch in range(number_of_switches):
            x = random.uniform(self.x_range[0], self.x_range[1])
            y = random.uniform(self.y_range[0], self.y_range[1])
            locations.append((x, y))
        return locations

    def initialize_distances(self) -> list:
        distances = np.zeros((len(self.switches), len(self.switches)))
        for i, switch_i in enumerate(self.switches):
            for j, switch_j in enumerate(self.switches):
                if i != j:
                    xi, yi = self.locations[switch_i]
                    xj, yj = self.locations[switch_j]
                    distance = np.sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
                    distances[i][j] = distances[j][i] = distance
        return distances

    def print_instance(self) -> None:
        print(f'number_of_switches: {self.number_of_switches}, loads: {self.switches_loads}')