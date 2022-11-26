from gurobipy import *

def solve(stands,
          temperature,
          amount,
          alcohol_content,
          sugar,
          calorific_value,
          price,
          cup_type,
          people,
          budget,
          min_wine_total,
          max_wine_per_stand,
          min_avg_temperature):

    model = Model("christmas_market")

    # variable x
    x = {}
    for p in people:
        for s in stands:
            # modify vtype, lb, ub, obj. DO NOT change the name attribute, this is a requirement of tutOR.
            x[p, s] = model.addVar(name=f'x_{p}_{s}', lb=0, vtype=GRB.INTEGER)

    # TODO: Add potential additional variables.
    model.update()  # update to make the variables known

    # TODO: Add all contraints
    # personal budget
    for p in people:
        model.addConstr(quicksum(x[p, s] * price[s] for s in stands) <= budget[p])

    # personal preferences
    for p in people:
        for s in stands:
            model.addConstr(x[p, s] <= max_wine_per_stand[p,s])

    # minimum amount of wine
    for p in people:
        model.addConstr(quicksum(x[p, s] for s in stands) >= min_wine_total[p])

    # minimum average temperature
    for p in people:
        model.addConstr(quicksum(x[p, s] * (temperature[s] - min_avg_temperature) * amount[s] for s in stands) >= 0)

    # waiting time
    for p1 in people:
        for p2 in people:
            if p1 == p2:
                continue
            for s in stands:
                model.addConstr(x[p1, s] - x[p2, s] <= 1)
                model.addConstr(x[p2, s] - x[p1, s] <= 1)


    # objective
    model.setObjective(quicksum(x[p, s] * price[s] for p in people for s in stands), GRB.MINIMIZE)


    # update and optimization. DO NOT change code below this line in your submission
    model.update()
    model.optimize()

    # print solution
    if model.status == GRB.OPTIMAL:
        print(f'\n objective: {model.ObjVal}\n')
        for p in people:
            for s in stands:
                if int(x[p, s].x) >= 1:
                    print(f'{p} drinks {int(x[p, s].x)} cups at {s}')

    return model
