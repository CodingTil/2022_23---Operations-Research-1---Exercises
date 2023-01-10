from gurobipy import *
from em_net import solve
from em_net_visualize import display_solution

transport_methods = ['amb', 'hel']

locations = {
	'a': (50.24, -22.83),
	'b': (-10.18, -59.39),
	'c': (-19.39, -15.87),
	'd': (-90.54, -83.57),
	'e': (90.27, -92.78),
	'f': (-30.89, 45.25),
	'g': (11.45, -33.59),
	'h': (-39.63, -11.43),
	'i': (46.39, 55.22),
	'j': (-40.83, -35.26),
	'k': (85.79, 75.77),
	'l': (-36.91, 88.78),
	'm': (31.97, -33.32),
	'n': (-73.9, -40.08),
	'o': (-18.64, -91.79),
	'p': (-13.84, -57.85),
	'q': (-45.35, -12.0),
	'r': (-13.19, 94.0),
	's': (45.08, 6.23),
	't': (-45.34, 85.32),
	'u': (-27.73, -73.62),
	'v': (-26.31, -82.75),
	'w': (67.41, -23.62),
	'x': (-96.81, 71.69)
}

locations_hel = [
	'n',
	'v',
	'q',
	'a',
	'd',
	'k',
	'w',
	'e',
	'm',
	'p',
	'l',
	'o'
]

capacities = {
	'amb': 8,
	'hel': 2
}

max_supply = {
	'amb': 5,
	'hel': 2
}

velocities = {
	'amb': 36,
	'hel': 86
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
