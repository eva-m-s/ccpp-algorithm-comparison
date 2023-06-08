from gekko import GEKKO
import numpy as np
import random
import time

def sp(max_load, switch_loads, num_switches, d, r):
    # Set variables
    num_controllers = num_switches

    # Initialize Model
    m = GEKKO(remote=False)

    # Define the decision variables
    z = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)
    c = m.Array(m.Var, num_controllers, lb=0, ub=1, integer=True)

    # Initialize the variables
    for i in range(num_switches):
        for j in range(num_controllers):
            z[i, j] = m.Var(lb=0, ub=1, integer=True)
    for j in range(num_controllers):
        c[j] = m.Var(lb=0, ub=1, integer=True)

    # Define constraints
    # The loads on each controller cannot exceed its capacity
    for j in range(num_controllers):
        m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j] * c[j])

    # Each switch must be assigned to exactly one controller
    for i in range(num_switches):
        m.Equation(m.sum(z[i, :]) == 1)

    # Each controller can only be activated if it is assigned to at least one switch
    for j in range(num_controllers):
        m.Equation(m.sum(z[:, j]) <= num_switches * c[j])

    # Distance between each switch and its assigned controller must be less than or equal to r
    for i in range(num_switches):
        for j in range(num_controllers):
            m.Equation(d[i][j] * z[i, j] <= r)

    # Define the objective function to minimize the number of controllers used
    obj = m.sum(c)
    m.Obj(obj)

    # Set solver options
    m.options.SOLVER = 1
    m.options.IMODE = 3

    # Start time
    start_time = time.time()

    # Initialize an empty list to store used distances
    used_distances = []

    try:
        # Solve the Gekko model
        m.solve(disp=False)
        if m.options.APPSTATUS == 1:  # solution successful
            # End time
            end_time = time.time()
            # Calculate elapsed time in seconds
            elapsed_time = end_time - start_time
            # Iterate through the z array and check if z[i,j] is equal to 1
            for i in range(num_switches):
                for j in range(num_controllers):
                    if z[i, j].value[0] == 1:
                        # If switch i is assigned to controller j, append distance d[i][j] to the list
                        used_distances.append(d[i][j])

            # Convert the list of used distances to a numpy array if needed
            used_distances = np.array(used_distances)
            total_distance = np.sum(used_distances)
            print("Time: ", elapsed_time, " seconds")
            print('Distances array:', used_distances)
            print('Num of controllers: ', m.options.ObjFcnVal)
            print('Total distance:', total_distance)
            print('Z: ', z)
            print('C: ', c)
            return m.options.ObjFcnVal

    except Exception as e:
        print("Error: Solution not found.")
        print(e)
        # Take appropriate action when a solution is not found
        return None




