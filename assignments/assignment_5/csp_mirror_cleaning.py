from gurobipy import *

def solve(transport_arcs, cleaning_arcs, nodes, max_num_robots, water_capacity, max_cleaning_time):
    # setup model
    model = Model("csp_mirror_cleaning")

    all_arcs = {**transport_arcs, **cleaning_arcs}

    # setup variables
    x = {}
    for a in all_arcs:
        for k in range(max_num_robots):
            x[a, k] = model.addVar(name=f'x_{a}_{k}', vtype=GRB.BINARY)

    time_at_node = {}
    for n in nodes:
        for k in range(max_num_robots):
            time_at_node[n, k] = model.addVar(name=f'time_at_node_{n}_{k}',
                            vtype=GRB.CONTINUOUS, lb=0, ub=max_cleaning_time)

    # max_arc_time = 0
    # for a in all_arcs:
        # if all_arcs[a]['time'] > max_arc_time:
            # max_arc_time = all_arcs[a]['time']
    # TODO!!! (maybe max_arc_time necessary?)
    # M = max_cleaning_time + max_arc_time + 1 
    M = max_cleaning_time + 1 

    # constraints
    # speedup: limit max number of x[a,k]:
    # for k in range(max_num_robots):
        # model.addConstr(quicksum(x[a, k] for a in all_arcs) <= len(nodes))

    # Set time_at_node to 0 if not using an node (for performance speedup)
    # for k in range(max_num_robots):
        # for n in nodes:
            # model.addConstr(time_at_node[n, k] <= quicksum(x[a, k] for a in
                # all_arcs if a[0] == n or a[1] == n) * (M / 2))

    # Each robot performs a tour
    for k in range(max_num_robots):
        for n in nodes:
            model.addConstr(quicksum(x[a, k] for a in all_arcs if a[0] == n) -
                    quicksum(x[a, k] for a in all_arcs if a[1] == n) == 0)

    # All robots must start and end at i = 0
    for k in range(max_num_robots):
        model.addConstr(quicksum(x[a, k] for a in all_arcs) <= len(all_arcs) *
                        quicksum(x[a, k] for a in all_arcs if a[0] == 0))
        model.addConstr(quicksum(x[a, k] for a in all_arcs) <= len(all_arcs) *
                        quicksum(x[a, k] for a in all_arcs if a[1] == 0))

    # Each robot can only visit each arc and node at most one time
    for k in range(max_num_robots):
        # for a in all_arcs:
            # if (a[0] < a[1]):
                # model.addConstr(x[a, k] + x[(a[1], a[0]), k] <= 1)
        for n in nodes:
            model.addConstr(
                    quicksum(x[a, k] for a in all_arcs if a[0] == n) <= 1)
            model.addConstr(
                    quicksum(x[a, k] for a in all_arcs if a[1] == n) <= 1)


    # Row of mirrors needs to be visited exactly once by any of the robots
    for a in cleaning_arcs:
        if (a[0] < a[1]):
            model.addConstr(quicksum(x[a, k] + x[(a[1], a[0]), k] for k in
                                     range(max_num_robots)) == 1)

    # Time-tracking mechanism for Subtour eliminitation contraints
    for k in range(max_num_robots):
        for a in all_arcs:
            if a[0] != 0:
                model.addConstr(time_at_node[a[0], k] + all_arcs[a]['time'] <=
                            time_at_node[a[1], k] + M * (1 - x[a, k]))
            else:
                model.addConstr(all_arcs[a]['time'] <=
                            time_at_node[a[1], k] + M * (1 - x[a, k]))

    # If robot k is used then k-1 is also used:
    # => If robot k uses an outgoing edge from the start k-1 has to use one too
    # => If robot k uses an ingoing edge from the start k-1 has to use one too
    for k in range(1, max_num_robots):
        model.addConstr(quicksum(x[a, k-1] for a in all_arcs if a[0] == 0) >=
                            quicksum(x[a, k] for a in all_arcs if a[0] == 0))
        model.addConstr(quicksum(x[a, k-1] for a in all_arcs if a[1] == 0) >=
                            quicksum(x[a, k] for a in all_arcs if a[1] == 0))

    # Ensure maximum water capacity for all robots
    for k in range(max_num_robots):
        model.addConstr(quicksum(cleaning_arcs[a]['water'] * x[a, k]
                                 for a in cleaning_arcs) <= water_capacity)

    # Ensure maximum cleaning time
    for k in range(max_num_robots):
        model.addConstr(quicksum(all_arcs[a]['time'] * x[a, k]
                                 for a in all_arcs) <= max_cleaning_time)

    # objective
    # minimize total aggregated time spent outside by all robots
    model.setObjective(quicksum(time_at_node[0, k] for k in range(max_num_robots)),
                       GRB.MINIMIZE)

    # do not change the code below this line
    model.update()
    # model.write('model.lp') # you may comment this line in to have a look at the LP file
    model.optimize()
    return model, x
