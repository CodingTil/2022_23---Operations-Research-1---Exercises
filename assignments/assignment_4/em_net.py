from gurobipy import *

def prepare_data(locations, velocities):
    reaction_times = {}
    # Calculate reaction times for ambulances and helicopters. Hint: the visualization function expects
    # the format: reaction_times[i, j, t]
    for i in locations:
        for j in locations:
            amb_distance = abs(locations[i][0] - locations[j][0]) + abs(locations[i][1] - locations[j][1])
            reaction_times[i, j, "amb"] = round(amb_distance / velocities["amb"], 4)
            hel_distance = ((locations[i][0] - locations[j][0])**2 + (locations[i][1] - locations[j][1])**2)**0.5
            reaction_times[i, j, "hel"] = round(hel_distance / velocities["hel"], 4)
    return reaction_times


def solve(transport_methods, locations, locations_hel, capacities, max_supply, velocities):
    # calculate reaction times
    reaction_times = prepare_data(
        locations=locations,
        velocities=velocities
    )

    model = Model("emergency_netwORk")

    # variables
    x = {}
    y = {}
    quickest = {}
    for i in locations:
        for j in locations:
            for t in transport_methods:
                x[i, j, t] = model.addVar(name=f'x_{i}_{j}_{t}', vtype=GRB.BINARY)
                y[i, t] = model.addVar(name=f'y_{i}_{t}', vtype=GRB.BINARY)
                quickest[i, j, t] = model.addVar(name=f'quickest_{i}_{j}_{t}', vtype=GRB.BINARY)

    max_response_time = model.addVar(name="max_response_time", vtype=GRB.CONTINUOUS)

    # constraints
    # If you assign a location j to a hospital i with a transport method t in has to be deployed in that hospital
    for i in locations:
        for j in locations:
            for t in transport_methods:
                model.addConstr(x[i, j, t] <= y[i, t])

    # A transport method can only be the quickest if it is assigned to a location
    for i in locations:
        for j in locations:
            for t in transport_methods:
                model.addConstr(quickest[i, j, t] <= x[i, j, t])

    # Every assigned location can have at most one quickest transport method
    for i in locations:
        for j in locations:
            model.addConstr(quicksum(quickest[i, j, t] for t in transport_methods) <= 1)

    # Every location j needs to be assigned to at least one hospital in location i by at least one method of transport
    for j in locations:
        model.addConstr(quicksum(x[i, j, t] for i in locations for t in transport_methods) >= 1)

    # Every location j needs to be assigned to at least one hospital in location i by at least one quickest method of transport
    for j in locations:
        model.addConstr(quicksum(quickest[i, j, t] for i in locations for t in transport_methods) >= 1)

    # Helicopters can only be deployed in a location i if ambulances are also deployed in the same location i
    for i in locations:
        model.addConstr(y[i, "hel"] <= y[i, "amb"])

    for j in locations:
        if j not in locations_hel:
            # A location j not in L_hel cannot be assigned to a hospital by means of a helicopter
            model.addConstr(quicksum(x[i, j, "hel"] for i in locations) == 0)

            # Helicopters can only be deployed in a hospital if they are also able to land there
            model.addConstr(y[j, "hel"] == 0)

    # There is a maximum number of locations p_t in which the method of transportation t can be deployed
    for t in transport_methods:
        model.addConstr(quicksum(y[i, t] for i in locations) <= max_supply[t])

    # The number of assigned locations of a hospital cannot exceed its aggregated capacity
    for i in locations:
        model.addConstr(quicksum(x[i, j, t] for j in locations for t in transport_methods) <= quicksum(y[i, t] * capacities[t] for t in transport_methods))

    # The response time of the quickest transport method has to be smaller than the max response time
    for i in locations:
        for j in locations:
            for t in transport_methods:
                model.addConstr(quickest[i, j, t] * reaction_times[i, j, t] <= max_response_time)

    # objective function
    # Minimize the maximum reaction time over all location assignments and all method of transport
    model.setObjective(max_response_time, GRB.MINIMIZE)

    # update and optimization. DO NOT change code below this line in your submission
    model.update()
    # model.write('model.lp') # you may comment in this line to see the model in the LP file
    model.optimize()

    return model, x, y, reaction_times



