from gurobipy import *


def solve(
    transport_arcs,
    cleaning_arcs,
    nodes,
    max_num_robots,
    water_capacity,
    max_cleaning_time,
):
    # setup model
    model = Model("csp_mirror_cleaning")

    A = transport_arcs.copy()
    A.update(cleaning_arcs)

    # setup variables
    x = {}
    t = {}
    for k in range(max_num_robots):
        # x_a,k​ is equal to 1 iff arc a is used by robot k and 0 otherwise
        for a in A:
            x[a, k] = model.addVar(
                name=f"x_{a}_{k}", vtype=GRB.BINARY
            )  # TODO: change attributes if needed. Do not change the name.
            # TODO: arcs are assumed to have the form a=(i,j) [no spaces!]

        for i in nodes:
            # t_i,k​ is the time when robot k reaches node i
            t[i, k] = model.addVar(
                name=f"t_{i}_{k}", vtype=GRB.CONTINUOUS, lb=0)

    # constraints
    # TODO: add constraints here

    # Each node is visited at most once
    for k in range(max_num_robots):
        for i in nodes:
            model.addConstr(quicksum(x[a, k] for a in A if a[0] == i) <= 1)
            # model.addConstr(quicksum(x[a, k] for a in A if a[1] == i) <= 1) # reduntant
            model.addConstr(quicksum(x[a, k] for a in A if a[0] == i) == quicksum(x[a, k] for a in A if a[1] == i))

    # All mirrors need to be cleaned exactly once
    for a in cleaning_arcs:
        if a[0] < a[1]:
            model.addConstr(quicksum(x[a, k] + x[(a[1], a[0]), k] for k in range(max_num_robots)) == 1)

    # Time tracking
    M = sum([max(A[a]['time'], A[(a[1], a[0])]['time']) for a in A if a[0] < a[1]])
    for k in range(max_num_robots):
        for a in A:
            if a[0] == 0:
                model.addConstr(t[a[1], k] + M * (1 - x[a, k]) >= A[a]['time'])
            else:
                model.addConstr(t[a[1], k] + M * (1 - x[a, k]) >= t[a[0], k] + A[a]['time'])

    # Cleaning last mirror has to be completed before time limit
    M = sum([max(A[a]['time'], A[(a[1], a[0])]['time']) for a in A if a[0] < a[1]]) - max_cleaning_time
    for k in range(max_num_robots):
        for a in cleaning_arcs:
            model.addConstr(t[a[1], k] <= max_cleaning_time + M * (1 - x[a, k]))

    # Water
    for k in range(max_num_robots):
        model.addConstr(quicksum(x[a, k] * A[a]['water'] for a in cleaning_arcs) <= water_capacity)

    # Break symmetry
    for k in range(max_num_robots - 1):
        model.addConstr(t[0, k] >= t[0, k + 1])

    # Add objective
    # model.setObjective(quicksum(t[0, k] for k in range(max_num_robots)), GRB.MINIMIZE)
    model.setObjective(quicksum(x[a, k] * A[a]['time'] for k in range(max_num_robots) for a in A), GRB.MINIMIZE)

    # do not change the code below this line
    model.update()
    # model.write('model.lp') # you may comment this line in to have a look at the LP file
    model.optimize()

    return model, x
