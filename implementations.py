from gekko import GEKKO
import numpy as np
from sp import sp
from sp_lr import sp_lr


def geeko_implementation(num_switches, num_controllers, max_load, switch_loads, distances, k):
    # Set variables
    num_controllers = num_switches
    # Take k maximum values from max_load and sum them
    max_sum = np.sum(np.sort(max_load)[-k:])

    # Sum the elements of switch_loads
    switch_sum = np.sum(switch_loads)

    # Compare the two sums
    if switch_sum > max_sum:
        #print("The sum of loads needed to control the switches exceeds controllers capacity. Problem cannot be solved.")
        print("Condition fail")
        return None, None
    else:
        # Initialize Model
        m = GEKKO(remote=False)

        # Define the decision variables
        z = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)
        c = m.Array(m.Var, num_controllers, lb=0, ub=1, integer=True)

        # Initialize a binary variable z[i, j]
        for i in range(num_switches):
            for j in range(num_controllers):
                z[i, j] = m.Var(lb=0, ub=1, integer=True)

        # Initialize a binary variable c[j]
        for j in range(num_controllers):
            c[j] = m.Var(lb=0, ub=1, integer=True)

        # Define constrains
        # The loads on each controller cannot exceed its capacity
        for j in range(num_controllers):
            m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j] * c[j])

        # Each switch must be assigned to exactly one controller
        for i in range(num_switches):
            m.Equation(m.sum(z[i, :]) == 1)

        # Each controller can only be activated if it is assigned to at least one switch
        for j in range(num_controllers):
            m.Equation(m.sum(z[:, j]) <= num_switches * c[j])

        # Number of active controllers must be equal or lower that k
        m.Equation(m.sum(c) <= k)

        # # Define the objective function
        # obj = 0
        # for i in range(num_switches):
        #     for j in range(num_controllers):
        #         for k in range(num_switches):
        #             obj += m.max2(m.sum([distances[i][k] * p[k][j] * z[i][j]]), 0)
        # Initialize the sum variable
        obj = 0.0

        for j in range(num_controllers):
            for i in range(num_switches):
                obj += m.max2(m.sum([distances[i][j] * z[i][j] * c[j]]), 0)

        m.Obj(obj)
        m.options.SOLVER = 1

        try:
            m.solve(disp=False)
        except Exception as e:
            # print("Exception")
            return None, None
            # print(e)
            # print("Solution not found")
        if m.options.APPSTATUS == 1:  # solution successful
            c_sum = []
            for i in list(c):
                c_sum.append(*i)
            # print("Objective function value =", m.options.ObjFcnVal)
            # print("Switches assignment: \n", z)
            # print("\nControllers state: \n", c)
            return m.options.ObjFcnVal, c_sum.count(1.0)


def article_implementation(num_switches, max_load_algorithm, switch_loads, distances, k):
    # Step 1: Sort distances in ascending order
    # Flatten the distance matrix into a 1D array
    dis_array = distances.flatten()
    # Sort the 1D array in ascending order
    dis_array = np.sort(dis_array)
    # Step 2: Binary search for the minimum radius r
    min_radius = None
    min_controllers = None
    lower = 0
    upper = len(dis_array) - 1

    while lower < upper:
        mid = (lower + upper) // 2
        r = dis_array[mid]
        num_controllers = sp_lr(max_load_algorithm, switch_loads, num_switches, distances, r, k)

        if num_controllers is not None:
            if num_controllers > k:
                lower = mid + 1

            else:
                min_radius = r
                min_controllers = num_controllers
                upper = mid
        else:

            # lower = mid + 1
            break



    # Step 3: Find placement for minimum radius
    print(f"STEP 3 {min_radius} {num_controllers}")
    min_radius2 = None
    min_controllers2 = k + 1
    index = lower
    num_controllers = sp(max_load_algorithm, switch_loads, num_switches, distances, dis_array[index], k)

    if num_controllers is not None:
        print(num_controllers, k)
        while num_controllers > k:
            index += 1
            if index == len(dis_array):
                print(f"Index issue")
                break
            num_controllers = sp(max_load_algorithm, switch_loads, num_switches, distances, dis_array[index], k)
        return min_radius, num_controllers

    else:
        return None, None
