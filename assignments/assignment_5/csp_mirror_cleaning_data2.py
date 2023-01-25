from gurobipy import *
from csp_mirror_cleaning import solve
from csp_mirror_cleaning_visualize import display_solution

nodes = {
	0: (0, 0),
	1: (0, 30),
	2: (7, 30),
	3: (80.64, 30),
	4: (0, 40),
	5: (7, 40),
	6: (80.64, 40),
	7: (0, 50),
	8: (7, 50),
	9: (80.64, 50),
	10: (87.64, 30),
	11: (166.88, 30),
	12: (87.64, 40),
	13: (166.88, 40),
	14: (87.64, 50),
	15: (166.88, 50),
	16: (173.88, 30),
	17: (241.89, 30),
	18: (173.88, 40),
	19: (241.89, 40),
	20: (173.88, 50),
	21: (241.89, 50),
	22: (248.89, 30),
	23: (314.74, 30),
	24: (248.89, 40),
	25: (314.74, 40),
	26: (248.89, 50),
	27: (314.74, 50),
	28: (321.74, 30),
	29: (321.74, 40),
	30: (321.74, 50)
}

cleaning_arcs = {
	(2, 3): {'time': 1261.01, 'water': 115.67, 'closing': False},
	(3, 2): {'time': 1180.98, 'water': 115.67, 'closing': False},
	(5, 6): {'time': 1639.63, 'water': 154.23, 'closing': False},
	(6, 5): {'time': 1616.37, 'water': 154.23, 'closing': False},
	(10, 11): {'time': 1392.96, 'water': 124.47, 'closing': False},
	(11, 10): {'time': 1234.73, 'water': 124.47, 'closing': False},
	(12, 13): {'time': 1794.54, 'water': 165.96, 'closing': False},
	(13, 12): {'time': 1709.06, 'water': 165.96, 'closing': False},
	(16, 17): {'time': 1245.23, 'water': 106.83, 'closing': False},
	(17, 16): {'time': 1010.07, 'water': 106.83, 'closing': False},
	(18, 19): {'time': 1578.08, 'water': 142.44, 'closing': False},
	(19, 18): {'time': 1428.98, 'water': 142.44, 'closing': False},
	(22, 23): {'time': 1171.14, 'water': 103.44, 'closing': False},
	(23, 22): {'time': 1012.53, 'water': 103.44, 'closing': False},
	(24, 25): {'time': 1477.19, 'water': 137.92, 'closing': False},
	(25, 24): {'time': 1434.37, 'water': 137.92, 'closing': False},
	(28, 1): {'time': 5360.85, 'water': 505.39, 'closing': True},
	(1, 28): {'time': 5308.45, 'water': 505.39, 'closing': True},
	(29, 4): {'time': 7841.04, 'water': 673.85, 'closing': True},
	(4, 29): {'time': 6384.7, 'water': 673.85, 'closing': True}
}

transport_arcs = {
	(0, 1): {'time': 62.02, 'water': 0, 'closing': False},
	(1, 0): {'time': 51.98, 'water': 0, 'closing': False},
	(0, 2): {'time': 62.19, 'water': 0, 'closing': False},
	(2, 0): {'time': 51.81, 'water': 0, 'closing': False},
	(1, 2): {'time': 7.13, 'water': 0, 'closing': False},
	(2, 1): {'time': 6.8, 'water': 0, 'closing': False},
	(2, 5): {'time': 20.23, 'water': 0, 'closing': False},
	(5, 2): {'time': 17.77, 'water': 0, 'closing': False},
	(1, 4): {'time': 19.4, 'water': 0, 'closing': False},
	(4, 1): {'time': 18.6, 'water': 0, 'closing': False},
	(4, 5): {'time': 9.48, 'water': 0, 'closing': False},
	(5, 4): {'time': 9.09, 'water': 0, 'closing': False},
	(5, 8): {'time': 19.4, 'water': 0, 'closing': False},
	(8, 5): {'time': 18.6, 'water': 0, 'closing': False},
	(4, 7): {'time': 19.13, 'water': 0, 'closing': False},
	(7, 4): {'time': 18.87, 'water': 0, 'closing': False},
	(7, 8): {'time': 11.98, 'water': 0, 'closing': False},
	(8, 7): {'time': 11.23, 'water': 0, 'closing': False},
	(8, 9): {'time': 134.59, 'water': 0, 'closing': False},
	(9, 8): {'time': 109.61, 'water': 0, 'closing': False},
	(0, 3): {'time': 62.13, 'water': 0, 'closing': False},
	(3, 0): {'time': 51.87, 'water': 0, 'closing': False},
	(0, 10): {'time': 59.23, 'water': 0, 'closing': False},
	(10, 0): {'time': 54.77, 'water': 0, 'closing': False},
	(3, 10): {'time': 7.21, 'water': 0, 'closing': False},
	(10, 3): {'time': 6.71, 'water': 0, 'closing': False},
	(10, 12): {'time': 20.2, 'water': 0, 'closing': False},
	(12, 10): {'time': 17.8, 'water': 0, 'closing': False},
	(3, 6): {'time': 20.13, 'water': 0, 'closing': False},
	(6, 3): {'time': 17.87, 'water': 0, 'closing': False},
	(6, 12): {'time': 10.03, 'water': 0, 'closing': False},
	(12, 6): {'time': 8.54, 'water': 0, 'closing': False},
	(12, 14): {'time': 19.26, 'water': 0, 'closing': False},
	(14, 12): {'time': 18.74, 'water': 0, 'closing': False},
	(6, 9): {'time': 19.64, 'water': 0, 'closing': False},
	(9, 6): {'time': 18.36, 'water': 0, 'closing': False},
	(9, 14): {'time': 11.74, 'water': 0, 'closing': False},
	(14, 9): {'time': 11.47, 'water': 0, 'closing': False},
	(14, 15): {'time': 135.65, 'water': 0, 'closing': False},
	(15, 14): {'time': 127.12, 'water': 0, 'closing': False},
	(0, 11): {'time': 59.5, 'water': 0, 'closing': False},
	(11, 0): {'time': 54.5, 'water': 0, 'closing': False},
	(0, 16): {'time': 61.5, 'water': 0, 'closing': False},
	(16, 0): {'time': 52.5, 'water': 0, 'closing': False},
	(11, 16): {'time': 7.45, 'water': 0, 'closing': False},
	(16, 11): {'time': 6.48, 'water': 0, 'closing': False},
	(16, 18): {'time': 19.44, 'water': 0, 'closing': False},
	(18, 16): {'time': 18.56, 'water': 0, 'closing': False},
	(11, 13): {'time': 20.4, 'water': 0, 'closing': False},
	(13, 11): {'time': 17.6, 'water': 0, 'closing': False},
	(13, 18): {'time': 9.93, 'water': 0, 'closing': False},
	(18, 13): {'time': 8.64, 'water': 0, 'closing': False},
	(18, 20): {'time': 19.76, 'water': 0, 'closing': False},
	(20, 18): {'time': 18.24, 'water': 0, 'closing': False},
	(13, 15): {'time': 19.87, 'water': 0, 'closing': False},
	(15, 13): {'time': 18.13, 'water': 0, 'closing': False},
	(15, 20): {'time': 12.06, 'water': 0, 'closing': False},
	(20, 15): {'time': 11.16, 'water': 0, 'closing': False},
	(20, 21): {'time': 116.54, 'water': 0, 'closing': False},
	(21, 20): {'time': 108.99, 'water': 0, 'closing': False},
	(0, 17): {'time': 60.64, 'water': 0, 'closing': False},
	(17, 0): {'time': 53.36, 'water': 0, 'closing': False},
	(0, 22): {'time': 60.97, 'water': 0, 'closing': False},
	(22, 0): {'time': 53.03, 'water': 0, 'closing': False},
	(17, 22): {'time': 7.44, 'water': 0, 'closing': False},
	(22, 17): {'time': 6.49, 'water': 0, 'closing': False},
	(22, 24): {'time': 20.34, 'water': 0, 'closing': False},
	(24, 22): {'time': 17.66, 'water': 0, 'closing': False},
	(17, 19): {'time': 19.4, 'water': 0, 'closing': False},
	(19, 17): {'time': 18.6, 'water': 0, 'closing': False},
	(19, 24): {'time': 9.69, 'water': 0, 'closing': False},
	(24, 19): {'time': 8.88, 'water': 0, 'closing': False},
	(24, 26): {'time': 19.32, 'water': 0, 'closing': False},
	(26, 24): {'time': 18.68, 'water': 0, 'closing': False},
	(19, 21): {'time': 20.82, 'water': 0, 'closing': False},
	(21, 19): {'time': 17.18, 'water': 0, 'closing': False},
	(21, 26): {'time': 11.82, 'water': 0, 'closing': False},
	(26, 21): {'time': 11.39, 'water': 0, 'closing': False},
	(26, 27): {'time': 114.28, 'water': 0, 'closing': False},
	(27, 26): {'time': 104.08, 'water': 0, 'closing': False},
	(0, 23): {'time': 62.2, 'water': 0, 'closing': True},
	(23, 0): {'time': 51.8, 'water': 0, 'closing': True},
	(0, 28): {'time': 59.41, 'water': 0, 'closing': True},
	(28, 0): {'time': 54.59, 'water': 0, 'closing': True},
	(23, 28): {'time': 7.0, 'water': 0, 'closing': True},
	(28, 23): {'time': 6.93, 'water': 0, 'closing': True},
	(28, 29): {'time': 19.24, 'water': 0, 'closing': True},
	(29, 28): {'time': 18.76, 'water': 0, 'closing': True},
	(23, 25): {'time': 19.68, 'water': 0, 'closing': True},
	(25, 23): {'time': 18.32, 'water': 0, 'closing': True},
	(25, 29): {'time': 9.75, 'water': 0, 'closing': True},
	(29, 25): {'time': 8.82, 'water': 0, 'closing': True},
	(29, 30): {'time': 19.89, 'water': 0, 'closing': True},
	(30, 29): {'time': 18.11, 'water': 0, 'closing': True},
	(25, 27): {'time': 19.23, 'water': 0, 'closing': True},
	(27, 25): {'time': 18.77, 'water': 0, 'closing': True},
	(27, 30): {'time': 11.82, 'water': 0, 'closing': True},
	(30, 27): {'time': 11.39, 'water': 0, 'closing': True},
	(30, 7): {'time': 575.53, 'water': 0, 'closing': True},
	(7, 30): {'time': 491.4, 'water': 0, 'closing': True}
}

max_num_robots = 3
max_cleaning_time = 28800
water_capacity = 800

model, x = solve(
	nodes=nodes,
	cleaning_arcs=cleaning_arcs,
	transport_arcs=transport_arcs,
	max_num_robots=max_num_robots,
	max_cleaning_time=max_cleaning_time,
	water_capacity=water_capacity
)

if model.status == GRB.OPTIMAL:
	display_solution(
		nodes=nodes,
		cleaning_arcs=cleaning_arcs,
		transport_arcs=transport_arcs,
		x=x
	)
