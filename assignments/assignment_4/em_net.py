from gurobipy import *
import math


def prepare_data(locations, velocities):
    reaction_times = {}
    # TODO: Calculate reaction times for ambulances and helicopters. Hint: the visualization function expects
    # TODO: the format: reaction_times[i, j, t]
    # modes_of_transport = ['amb', 'hel']
    for a, a_loc in locations.items():
        for b, b_loc in locations.items():
            # for 'amb' calculate manhattan distance |x1 - x2| + |y1 - y2|
            manhattan_distance = abs(a_loc[0] - b_loc[0]) + abs(a_loc[1] - b_loc[1])
            # for 'hel' calculate euclidean distance sqrt((x1 - x2)^2 + (y1 - y2)^2)
            euclidean_distance = math.sqrt(
                (a_loc[0] - b_loc[0]) ** 2 + (a_loc[1] - b_loc[1]) ** 2
            )
            # calculate the response times, round to 4 digits
            manhattan_response_time = round(manhattan_distance / velocities["amb"], 4)
            euclidean_response_time = round(euclidean_distance / velocities["hel"], 4)
            reaction_times[a, b, "amb"] = manhattan_response_time
            reaction_times[a, b, "hel"] = euclidean_response_time
    return reaction_times


def solve(
    transport_methods, locations, locations_hel, capacities, max_supply, velocities
):
    # calculate reaction times  # TODO: Calculate reaction times within the function above
    reaction_times = prepare_data(locations=locations, velocities=velocities)

    model = Model("emergency_netwORk")

    # variables
    x = {}
    y = {}
    z = 0
    for a, _ in locations.items():
        for b, _ in locations.items():
            for t in transport_methods:
                # TODO: Implement a logic to define the variables x and y. Add attributes where needed. DO NOT change the name
                x[a, b, t] = model.addVar(name=f"x_{a}_{b}_{t}", vtype=GRB.BINARY)
                y[a, t] = model.addVar(name=f"y_{a}_{b}", vtype=GRB.BINARY)

    # TODO: Add additional variables if needed for your model
    z = model.addVar(name="z", vtype=GRB.CONTINUOUS)

    # constraints
    # extra: couple x and y
    for a, _ in locations.items():
        for b, _ in locations.items():
            for t in transport_methods:
                model.addConstr(x[a, b, t] <= y[a, t])

    # TODO: Add your constraints below
    # every location needs to be assigned to at least one hospital in location i by at least one method of transportation
    for b, _ in locations.items():
        model.addConstr(
            quicksum(
                x[a, b, t] for a, _ in locations.items() for t in transport_methods
            )
            >= 1
        )

    # if helicopters in i, then ambulances in i
    # every opened hospital also comes with the deployment of ambulances
    for a, _ in locations.items():
        model.addConstr(y[a, "amb"] >= y[a, "hel"])

    # helicopters can only land in specific locations
    # helicopters can only be deployed in a hospital if they are also able to land there
    for a, _ in locations.items():
        if a not in locations_hel:
            model.addConstr(y[a, "hel"] == 0)

    # there is a maximum number of locations in which the method of transportation can be deployed
    for t in transport_methods:
        model.addConstr(
            quicksum(y[a, t] for a, _ in locations.items()) <= max_supply[t]
        )

    # the number of assigned locations of a hospital cannot exceed its aggregated capacity
    for a, _ in locations.items():
        model.addConstr(
            quicksum(
                x[a, b, t] for b, _ in locations.items() for t in transport_methods
            )
            <= quicksum(y[a, t] * capacities[t] for t in transport_methods)
        )

    # objective function
    # TODO Add your objective function here (or do that with the variable definition)
    # minimize the maximum response time (min (arg max))
    for a, _ in locations.items():
        for b, _ in locations.items():
            for t in transport_methods:
                model.addConstr(x[a, b, t] * reaction_times[a, b, t] <= z)
    model.setObjective(z, GRB.MINIMIZE)

    # update and optimization. DO NOT change code below this line in your submission
    model.update()
    # model.write('model.lp') # you may comment in this line to see the model in the LP file
    model.optimize()

    # print solution
    if model.status == GRB.OPTIMAL:
        print(f"\n objective: {model.ObjVal}\n")
        for a, _ in locations.items():
            val = 0
            for b, _ in locations.items():
                for t in transport_methods:
                    val += x[a, b, t].x
            if val <= 0 and y[a, "amb"].x > 0:
                print(f"x_{a}_{b} = {val}")
            if y[a, "hel"].x > y[a, "amb"].x:
                print(f"y_{a} = {y[a, 'hel'].x}")
            if y[a, "hel"].x > 0 and not a in locations_hel:
                print(f"y_{a} = {y[a, 'hel'].x}")

    return model, x, y, reaction_times
