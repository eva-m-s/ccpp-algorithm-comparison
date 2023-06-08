from gekko import GEKKO


def sdn_opt(num_switches, num_controllers, max_load, switch_loads, distances):

    # Initialize Model
    m = GEKKO(remote=False)

    # Define the decision variables
    z = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)
    p = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)

    # Initialize a binary variable z[i, j]
    for i in range(num_switches):
        for j in range(num_controllers):
            z[i, j] = m.Var(lb=0, ub=1, integer=True)

    # Initialize a binary variable p[i, j]
    for i in range(num_switches):
        for j in range(num_controllers):
            p[i, j] = m.Var(lb=0, ub=1, integer=True)

    # Each switch must be assigned to exactly one controller
    for i in range(num_switches):
        m.Equation(m.sum(z[i, :]) == 1)

    # The loads on each controller cannot exceed its capacity
    for j in range(num_controllers):
        m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j])

    # Each controller must be placed at exactly one location
    for j in range(num_controllers):
        m.Equation(m.sum([p[i, j] for i in range(num_switches)]) == 1)

    # Each location for controller can only be used once
    for i in range(num_switches):
        m.Equation(m.sum([p[i, j] for j in range(num_controllers)]) <= 1)

    # Define the objective function
    obj = 0
    for i in range(num_switches):
        for j in range(num_controllers):
            for k in range(num_switches):
                obj += m.max2(m.sum([distances[i][k] * p[k][j] * z[i][j]]), 0)

    m.Obj(obj)
    m.options.SOLVER = 1

    try:
        m.solve(disp=False)
        if m.options.APPSTATUS == 1:  # solution successful
            print("Objective function value =", m.options.ObjFcnVal)
            print("Switches assignment: \n", z)
            print("\nControllers placement: \n", p)

    except Exception as e:
        print(e)
        print("Solution not found")


