from gurobipy import *


def prepare_data(groups, payment_methods):
    # build list of all people and dictionary of their net balances
    people = []
    balances = {}
    for group in groups:
        for person in group:
            if person not in people:
                people.append(person)
                balances[person] = group[person]
            else:
                balances[person] = balances[person] + group[person]

    # build dictionaries for looking up friends and a dictionary for shared methods between friends
    friends = {p: [] for p in people}
    shared_methods = {}
    for person in friends:
        for group in groups:
            if person in group:
                for pot_friend in group:
                    if not pot_friend == person:
                        if pot_friend not in friends[person]:
                            friends[person].append(pot_friend)
                            shared_methods[(person,pot_friend)] = [m for m in payment_methods[person] if m in
                                                                   payment_methods[pot_friend]]
    return people, friends, balances, shared_methods


def solve(groups, max_transactions_pp, payment_methods):
    # retrieve converted data
    people, friends, balances, shared_methods = prepare_data(groups, payment_methods)

    model = Model("splitmORe")

    # set big M as sum of all balances + 1
    M = sum([abs(balances[p]) for p in people]) + 1

    # setup variables
    x = {}
    for p1 in people: # TODO: Add a logic that defines the variables for the cases that we need
        for p2 in friends[p1]:
            for m in shared_methods[(p1, p2)]:
                # amount of money transferred from p1 to p2 via m
                # gurobi type is continuous
                x[p1, p2, m] = model.addVar(name=f'x_{p1}_{p2}_{m}', vtype=GRB.CONTINUOUS, lb=0) # TODO: Add attributes if needed. Do not alter the name.

    # TODO: Add additional variables in this section if you need them for your model
    y = {}
    for p1 in people:
        for p2 in friends[p1]:
            for m in shared_methods[(p1, p2)]:
                # whether x[p1, p2, m] is used
                y[p1, p2, m] = model.addVar(name=f'y_{p1}_{p2}_{m}', vtype=GRB.BINARY)

    # constraints
    # TODO: Define the constraints of the model here

    # A transaction must not be higher than the maximum absolute value of the net balances of the two respective transaction partners p1p1​ and p2p2​.
    for p1 in people:
        for p2 in friends[p1]:
            for m in shared_methods[(p1, p2)]:
                model.addConstr(x[p1, p2, m] <= max(abs(balances[p1]), abs(balances[p2])))

    # y=1 -> x>0, y=0 -> x=0
    for p1 in people:
        for p2 in friends[p1]:
            for m in shared_methods[(p1, p2)]:
                model.addConstr(0 <= x[p1, p2, m] + M * (1 - y[p1, p2, m]))
                model.addConstr(x[p1, p2, m] <= M * y[p1, p2, m])

    # between two people, at most one method of payment may be used
    for p1 in people:
        for p2 in friends[p1]:
            model.addConstr(quicksum(y[p1, p2, m] for m in shared_methods[(p1, p2)]) <= 1)

    # resulting balances are zero for all people
    for p1 in people:
        model.addConstr(-quicksum(x[p1, p2, m] for p2 in friends[p1] for m in shared_methods[(p1, p2)]) + quicksum(x[p2, p1, m] for p2 in friends[p1] for m in shared_methods[(p1, p2)]) == balances[p1])

    # a single person must not make more than max_transactions_pp transactions
    for p1 in people:
        model.addConstr(quicksum(y[p1, p2, m] for p2 in friends[p1] for m in shared_methods[(p1, p2)]) <= max_transactions_pp)

    # objective
    # TODO: Define the objective here (or above with variable definition)
    # the goal is to minimize the number of transactions
    model.setObjective(quicksum(y[p1, p2, m] for p1 in people for p2 in friends[p1] for m in shared_methods[(p1, p2)]), GRB.MINIMIZE)

    # update and optimization. DO NOT change code below this line in your submission
    model.update()
    # model.write('model.lp')
    model.optimize()

    # print solution
    if model.status == GRB.OPTIMAL:
        print(f'\nObjective: {model.ObjVal}\n')
        print(f'\n====== Balances =====')
        for p in people:
            print(f'{p} is at {round(balances[p],2)}')
        print('\n====== Flow =====')
        for p1 in people:
            for p2 in friends[p1]:
                if len(shared_methods[(p1, p2)]) > 0:
                    for m in shared_methods[(p1, p2)]:
                        if round(x[p1, p2, m].x,2) > 0:
                            print(f'{p1} gives {p2} {round(x[p1, p2, m].x,2)} EUR via {m}')

    return model
