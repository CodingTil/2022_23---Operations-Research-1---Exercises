from gurobipy import *
from em_net import solve
from em_net_visualize import display_solution

transport_methods = ['amb', 'hel']

locations = {
	'a': (97.83, 37.33),
	'b': (-12.26, -31.82),
	'c': (-71.41, 97.63),
	'd': (54.53, 35.5),
	'e': (73.0, 15.52),
	'f': (55.85, -93.99),
	'g': (-77.6, -75.06),
	'h': (32.48, -3.79),
	'i': (71.03, -76.43),
	'j': (-60.88, 11.24),
	'k': (-56.33, -36.65),
	'l': (42.62, 19.54),
	'm': (22.33, -47.42),
	'n': (-84.0, -2.68),
	'o': (-19.3, 58.74),
	'p': (88.36, 82.95),
	'q': (-63.56, 45.52),
	'r': (-29.65, 8.28),
	's': (-53.31, -3.24)
}

locations_hel = [
	'q',
	'c',
	'h',
	'm',
	's',
	'e',
	'p',
	'o',
	'd'
]

capacities = {
	'amb': 6,
	'hel': 1
}

max_supply = {
	'amb': 4,
	'hel': 1
}

velocities = {
	'amb': 36,
	'hel': 89
}

if __name__ == '__main__':
	model, x, y, reaction_times = solve(
		transport_methods=transport_methods,
		locations=locations,
		locations_hel=locations_hel,
		capacities=capacities,
		max_supply=max_supply,
		velocities=velocities,
	)
	if model.status == GRB.OPTIMAL:
		display_solution(
			x=x,
			y=y,
			locations=locations,
			transport_methods=transport_methods,
			reaction_times=reaction_times
		)
