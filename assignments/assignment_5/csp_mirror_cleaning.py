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
    use = {}
    t = {}
    node = {}
    for k in range(max_num_robots):
        # x_a,k​ is equal to 1 iff arc a is used by robot k and 0 otherwise
        for a in A:
            x[a, k] = model.addVar(
                name=f"x_{a}_{k}", vtype=GRB.BINARY
            )  # TODO: change attributes if needed. Do not change the name.
            # TODO: arcs are assumed to have the form a=(i,j) [no spaces!]

        # use_k robot k is used
        use[k] = model.addVar(name=f"use_{k}", vtype=GRB.BINARY)

        # t_i^k is the time a robot visits node i
        for i in nodes:
            t[i, k] = model.addVar(name=f"t_{i}^{k}", vtype=GRB.CONTINUOUS)
            node[i, k] = model.addVar(name=f"node_{i}^{k}", vtype=GRB.BINARY)

    # TODO: add additional variables here if needed

    # constraints
    # TODO: add constraints here
    # couple use_k, x_a,k​, z_i,k​
    for k in range(max_num_robots):
        for a in A:
            model.addConstr(x[a, k] <= use[k])
        for i in nodes:
            model.addConstr(node[i, k] <= use[k])

    # Each tour must begin and end at node 0 (only used robots)
    for k in range(max_num_robots):
        model.addConstr(quicksum(x[a, k] for a in A if a[0] == 0) == use[k])
        model.addConstr(quicksum(x[a, k] for a in A if a[1] == 0) == use[k])
        model.addConstr(node[0, k] == use[k])

    # Each robot can only visit each arc and node at most one time.
    for k in range(max_num_robots):
        for a in A:
            a_opposite = (a[1], a[0])
            model.addConstr(x[a, k] + x[a_opposite, k] <= 1)
        for i in nodes:
            model.addConstr(quicksum(x[a, k] for a in A if a[0] == i) <= 1)
            model.addConstr(quicksum(x[a, k] for a in A if a[1] == i) <= 1)
            model.addConstr(node[i, k] <= 1) # redundant

    # if arc a=(i,j) is used, then another arc a' is used, where a'=(x,i), and a''=(j,y), where x and y are nodes that are not i or j.
    for k in range(max_num_robots):
        for i in nodes:
            model.addConstr(
                quicksum(x[a, k] for a in A if a[0] == i) == node[i, k]
            )
            model.addConstr(
                quicksum(x[a, k] for a in A if a[1] == i) == node[i, k]
            )

    # Each row of mirrors needs to be visited exactly once by any of the robots.
    for a in cleaning_arcs:
        a_opposite = (a[1], a[0])
        model.addConstr(
            quicksum(x[a, k] + x[a_opposite, k] for k in range(max_num_robots)) == 1
        )

    # time constraint
    M = 2**32
    for k in range(max_num_robots):
        for a in A:
            if a[0] == 0:
                model.addConstr(t[a[1], k] + M * (1 - x[a, k]) >= A[a]["time"])
            else:
                model.addConstr(t[a[1], k] + M * (1 - x[a, k]) >= t[a[0], k] + A[a]["time"])

    # Cleaning of the last mirror needs to be finished after max_cleaning_time. The robots may leave the tower for their tours at t_0^k=0.
    for k in range(max_num_robots):
        model.addConstr(
            quicksum(x[a, k] * A[a]["time"] for a in cleaning_arcs) <= max_cleaning_time
        )

    # A cleaning arc cannot be used for transport, i.e., once a robot travels on a cleaning arc, it also is cleaning the row.
    # Each robot has a maximum cleaning water capacity of water_capacity[k]. Cleaning mirrors on cleaning arcs a∈A_clean requires w_a​ water.
    for k in range(max_num_robots):
        model.addConstr(
            quicksum(x[a, k] * cleaning_arcs[a]["water"] for a in cleaning_arcs)
            <= water_capacity
        )

    # To break some of the inherent symmetry, a robot k∈{1,…,m−1} can only be used if robot k−1 is also used.
    for k in range(1, max_num_robots):
        model.addConstr(use[k - 1] >= use[k])

    # Add objective
    # The goal is to minimize the total aggregated time spent outside by all robots. This is necessary due to the harsh weather conditions that is likely to quickly wear down the robots.
    model.setObjective(
        quicksum(t[0, k] * use[k] for k in range(max_num_robots)),
        GRB.MINIMIZE,
    )

    # do not change the code below this line
    model.update()
    # model.write('model.lp') # you may comment this line in to have a look at the LP file
    model.optimize()

    return model, x
