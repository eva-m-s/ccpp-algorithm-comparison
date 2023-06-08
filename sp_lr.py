from gekko import GEKKO
import numpy as np


def sp_lr(max_load, switch_loads, num_switches, d, r, k):
    # Set variables
    num_controllers = num_switches

    # Take k maximum values from max_load and sum them
    max_sum = np.sum(np.sort(max_load)[-k:])

    # Sum the elements of switch_loads
    switch_sum = np.sum(switch_loads)

    # Compare the two sums
    if switch_sum > max_sum:
        print("The sum of loads needed to control the switches exceeds controllers capacity. Problem cannot be solved.")
        return None
    else:
        # Initialize Model
        m = GEKKO(remote=False)

        # Define the decision variables
        z = m.Array(m.Var, (num_switches, num_controllers))
        c = m.Array(m.Var, num_controllers)

        # Initialize the variables
        for i in range(num_switches):
            for j in range(num_controllers):
                z[i, j] = m.Var(lb=0, ub=1)
        for j in range(num_controllers):
            c[j] = m.Var(lb=0, ub=1)

        # Define constraints
        # The loads on each controller cannot exceed its capacity
        for j in range(num_controllers):
            m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j] * c[j])

        # Each switch must be assigned to exactly one controller
        for i in range(num_switches):
            m.Equation(m.sum(z[i, :]) == 1)

        # Distance between each switch and its assigned controller must be less than or equal to r
        for i in range(num_switches):
            for j in range(num_controllers):
                m.Equation(d[i][j] * z[i, j] <= r)

        # Define the objective function to minimize the number of used controllers'
        obj = m.sum(c)
        m.Obj(obj)

        # Set solver options
        m.options.SOLVER = 1
        m.options.IMODE = 3

        try:
            # Solve the Gekko model
            m.solve(disp=False)
            if m.options.APPSTATUS == 1:  # solution successful
                # print("Objective function value =", m.options.ObjFcnVal)
                # print('Z: ', z)
                print('C: ', c)
                return m.options.ObjFcnVal

        except Exception as e:
            print(e)
            # Take appropriate action when a solution is not found
            return k+1