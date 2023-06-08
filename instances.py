import random
import numpy as np


class NetworkInstances:
    # Number of controllers
    K = 3

    # Number of switches
    S = 8

    # Controllers maximum capacity
    capacities_algorith = [1, 8, 1, 3, 3, 1, 1, 1]
    capacities_model = [20, 80, 1]


    # Loads to control switches
    loads = [10, 10, 10, 10, 10, 10, 10, 10]

    # Switches locations:

    # Create set of switches
    switches = list(range(S))

    # Define the range of x and y coordinates
    x_range = (0, 90)
    y_range = (0, 180)

    # Assign a location to each switch
    locations = {}
    for switch in switches:
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        locations[switch] = (x, y)

    # Distance matrix 2D array
    d = np.zeros((len(switches), len(switches)))
    for i, switch_i in enumerate(switches):
        for j, switch_j in enumerate(switches):
            if i != j:
                xi, yi = locations[switch_i]
                xj, yj = locations[switch_j]
                dist = np.sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
                d[i][j] = d[j][i] = dist






