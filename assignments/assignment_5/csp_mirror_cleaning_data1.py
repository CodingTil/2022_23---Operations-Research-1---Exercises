from gurobipy import *
from csp_mirror_cleaning import solve
from csp_mirror_cleaning_visualize import display_solution

nodes = {
	0: (0, 0),
	1: (0, 30),
	2: (7, 30),
	3: (73.11, 30),
	4: (0, 40),
	5: (7, 40),
	6: (73.11, 40),
	7: (0, 50),
	8: (7, 50),
	9: (73.11, 50),
	10: (0, 60),
	11: (7, 60),
	12: (73.11, 60),
	13: (80.11, 30),
	14: (160.23, 30),
	15: (80.11, 40),
	16: (160.23, 40),
	17: (80.11, 50),
	18: (160.23, 50),
	19: (80.11, 60),
	20: (160.23, 60),
	21: (167.23, 30),
	22: (252.10, 30),
	23: (167.23, 40),
	24: (252.10, 40),
	25: (167.23, 50),
	26: (252.10, 50),
	27: (167.23, 60),
	28: (252.10, 60),
	29: (259.1, 30),
	30: (332.45, 30),
	31: (259.1, 40),
	32: (332.45, 40),
	33: (259.1, 50),
	34: (332.45, 50),
	35: (259.1, 60),
	36: (332.45, 60),
	37: (339.45, 30),
	38: (339.45, 40),
	39: (339.45, 50),
	40: (339.45, 60)
}

cleaning_arcs = {
	(2, 3): {'time': 1153.21, 'water': 103.85, 'closing': False},
	(3, 2): {'time': 1039.08, 'water': 103.85, 'closing': False},
	(5, 6): {'time': 1534.23, 'water': 138.46, 'closing': False},
	(6, 5): {'time': 1388.82, 'water': 138.46, 'closing': False},
	(8, 9): {'time': 1833.64, 'water': 173.08, 'closing': False},
	(9, 8): {'time': 1820.18, 'water': 173.08, 'closing': False},
	(13, 14): {'time': 1399.51, 'water': 125.85, 'closing': False},
	(14, 13): {'time': 1257.37, 'water': 125.85, 'closing': False},
	(15, 16): {'time': 1957.32, 'water': 167.8, 'closing': False},
	(16, 15): {'time': 1585.18, 'water': 167.8, 'closing': False},
	(17, 18): {'time': 2362.22, 'water': 209.75, 'closing': False},
	(18, 17): {'time': 2065.91, 'water': 209.75, 'closing': False},
	(21, 22): {'time': 1463.33, 'water': 133.31, 'closing': False},
	(22, 21): {'time': 1351.06, 'water': 133.31, 'closing': False},
	(23, 24): {'time': 1883.61, 'water': 177.75, 'closing': False},
	(24, 23): {'time': 1868.92, 'water': 177.75, 'closing': False},
	(25, 26): {'time': 2446.66, 'water': 222.19, 'closing': False},
	(26, 25): {'time': 2244.0, 'water': 222.19, 'closing': False},
	(29, 30): {'time': 1291.04, 'water': 115.22, 'closing': False},
	(30, 29): {'time': 1141.34, 'water': 115.22, 'closing': False},
	(31, 32): {'time': 1662.04, 'water': 153.62, 'closing': False},
	(32, 31): {'time': 1581.13, 'water': 153.62, 'closing': False},
	(33, 34): {'time': 2224.75, 'water': 192.03, 'closing': False},
	(34, 33): {'time': 1829.21, 'water': 192.03, 'closing': False},
	(37, 1): {'time': 5862.37, 'water': 533.21, 'closing': True},
	(1, 37): {'time': 5394.22, 'water': 533.21, 'closing': True},
	(38, 4): {'time': 7694.63, 'water': 710.94, 'closing': True},
	(4, 38): {'time': 7314.15, 'water': 710.94, 'closing': True},
	(39, 7): {'time': 9980.7, 'water': 888.68, 'closing': True},
	(7, 39): {'time': 8780.28, 'water': 888.68, 'closing': True}
}

transport_arcs = {
	(0, 1): {'time': 60.09, 'water': 0, 'closing': False},
	(1, 0): {'time': 53.91, 'water': 0, 'closing': False},
	(0, 2): {'time': 60.71, 'water': 0, 'closing': False},
	(2, 0): {'time': 53.29, 'water': 0, 'closing': False},
	(1, 2): {'time': 7.48, 'water': 0, 'closing': False},
	(2, 1): {'time': 6.45, 'water': 0, 'closing': False},
	(2, 5): {'time': 20.81, 'water': 0, 'closing': False},
	(5, 2): {'time': 17.19, 'water': 0, 'closing': False},
	(1, 4): {'time': 20.57, 'water': 0, 'closing': False},
	(4, 1): {'time': 17.43, 'water': 0, 'closing': False},
	(4, 5): {'time': 9.91, 'water': 0, 'closing': False},
	(5, 4): {'time': 8.66, 'water': 0, 'closing': False},
	(5, 8): {'time': 19.55, 'water': 0, 'closing': False},
	(8, 5): {'time': 18.45, 'water': 0, 'closing': False},
	(4, 7): {'time': 20.36, 'water': 0, 'closing': False},
	(7, 4): {'time': 17.64, 'water': 0, 'closing': False},
	(7, 8): {'time': 11.61, 'water': 0, 'closing': False},
	(8, 7): {'time': 11.6, 'water': 0, 'closing': False},
	(8, 11): {'time': 19.71, 'water': 0, 'closing': False},
	(11, 8): {'time': 18.29, 'water': 0, 'closing': False},
	(7, 10): {'time': 19.26, 'water': 0, 'closing': False},
	(10, 7): {'time': 18.74, 'water': 0, 'closing': False},
	(10, 11): {'time': 14.17, 'water': 0, 'closing': False},
	(11, 10): {'time': 13.68, 'water': 0, 'closing': False},
	(11, 12): {'time': 142.08, 'water': 0, 'closing': False},
	(12, 11): {'time': 120.99, 'water': 0, 'closing': False},
	(0, 3): {'time': 60.28, 'water': 0, 'closing': False},
	(3, 0): {'time': 53.72, 'water': 0, 'closing': False},
	(0, 13): {'time': 58.98, 'water': 0, 'closing': False},
	(13, 0): {'time': 55.02, 'water': 0, 'closing': False},
	(3, 13): {'time': 7.44, 'water': 0, 'closing': False},
	(13, 3): {'time': 6.49, 'water': 0, 'closing': False},
	(13, 15): {'time': 20.08, 'water': 0, 'closing': False},
	(15, 13): {'time': 17.92, 'water': 0, 'closing': False},
	(3, 6): {'time': 19.74, 'water': 0, 'closing': False},
	(6, 3): {'time': 18.26, 'water': 0, 'closing': False},
	(6, 15): {'time': 9.36, 'water': 0, 'closing': False},
	(15, 6): {'time': 9.21, 'water': 0, 'closing': False},
	(15, 17): {'time': 20.52, 'water': 0, 'closing': False},
	(17, 15): {'time': 17.48, 'water': 0, 'closing': False},
	(6, 9): {'time': 20.5, 'water': 0, 'closing': False},
	(9, 6): {'time': 17.5, 'water': 0, 'closing': False},
	(9, 17): {'time': 11.73, 'water': 0, 'closing': False},
	(17, 9): {'time': 11.48, 'water': 0, 'closing': False},
	(17, 19): {'time': 20.15, 'water': 0, 'closing': False},
	(19, 17): {'time': 17.85, 'water': 0, 'closing': False},
	(9, 12): {'time': 19.89, 'water': 0, 'closing': False},
	(12, 9): {'time': 18.11, 'water': 0, 'closing': False},
	(12, 19): {'time': 14.33, 'water': 0, 'closing': False},
	(19, 12): {'time': 13.53, 'water': 0, 'closing': False},
	(19, 20): {'time': 163.21, 'water': 0, 'closing': False},
	(20, 19): {'time': 155.62, 'water': 0, 'closing': False},
	(0, 14): {'time': 57.14, 'water': 0, 'closing': False},
	(14, 0): {'time': 56.86, 'water': 0, 'closing': False},
	(0, 21): {'time': 60.54, 'water': 0, 'closing': False},
	(21, 0): {'time': 53.46, 'water': 0, 'closing': False},
	(14, 21): {'time': 7.52, 'water': 0, 'closing': False},
	(21, 14): {'time': 6.4, 'water': 0, 'closing': False},
	(21, 23): {'time': 20.4, 'water': 0, 'closing': False},
	(23, 21): {'time': 17.6, 'water': 0, 'closing': False},
	(14, 16): {'time': 20.05, 'water': 0, 'closing': False},
	(16, 14): {'time': 17.95, 'water': 0, 'closing': False},
	(16, 23): {'time': 9.72, 'water': 0, 'closing': False},
	(23, 16): {'time': 8.85, 'water': 0, 'closing': False},
	(23, 25): {'time': 19.45, 'water': 0, 'closing': False},
	(25, 23): {'time': 18.55, 'water': 0, 'closing': False},
	(16, 18): {'time': 20.24, 'water': 0, 'closing': False},
	(18, 16): {'time': 17.76, 'water': 0, 'closing': False},
	(18, 25): {'time': 12.64, 'water': 0, 'closing': False},
	(25, 18): {'time': 10.57, 'water': 0, 'closing': False},
	(25, 27): {'time': 19.58, 'water': 0, 'closing': False},
	(27, 25): {'time': 18.42, 'water': 0, 'closing': False},
	(18, 20): {'time': 19.71, 'water': 0, 'closing': False},
	(20, 18): {'time': 18.29, 'water': 0, 'closing': False},
	(20, 27): {'time': 14.29, 'water': 0, 'closing': False},
	(27, 20): {'time': 13.57, 'water': 0, 'closing': False},
	(27, 28): {'time': 173.42, 'water': 0, 'closing': False},
	(28, 27): {'time': 164.31, 'water': 0, 'closing': False},
	(0, 22): {'time': 60.84, 'water': 0, 'closing': False},
	(22, 0): {'time': 53.16, 'water': 0, 'closing': False},
	(0, 29): {'time': 57.22, 'water': 0, 'closing': False},
	(29, 0): {'time': 56.78, 'water': 0, 'closing': False},
	(22, 29): {'time': 7.0, 'water': 0, 'closing': False},
	(29, 22): {'time': 6.92, 'water': 0, 'closing': False},
	(29, 31): {'time': 20.1, 'water': 0, 'closing': False},
	(31, 29): {'time': 17.9, 'water': 0, 'closing': False},
	(22, 24): {'time': 20.47, 'water': 0, 'closing': False},
	(24, 22): {'time': 17.53, 'water': 0, 'closing': False},
	(24, 31): {'time': 9.71, 'water': 0, 'closing': False},
	(31, 24): {'time': 8.86, 'water': 0, 'closing': False},
	(31, 33): {'time': 20.56, 'water': 0, 'closing': False},
	(33, 31): {'time': 17.44, 'water': 0, 'closing': False},
	(24, 26): {'time': 19.28, 'water': 0, 'closing': False},
	(26, 24): {'time': 18.72, 'water': 0, 'closing': False},
	(26, 33): {'time': 12.51, 'water': 0, 'closing': False},
	(33, 26): {'time': 10.7, 'water': 0, 'closing': False},
	(33, 35): {'time': 20.11, 'water': 0, 'closing': False},
	(35, 33): {'time': 17.89, 'water': 0, 'closing': False},
	(26, 28): {'time': 19.51, 'water': 0, 'closing': False},
	(28, 26): {'time': 18.49, 'water': 0, 'closing': False},
	(28, 35): {'time': 14.49, 'water': 0, 'closing': False},
	(35, 28): {'time': 13.37, 'water': 0, 'closing': False},
	(35, 36): {'time': 151.44, 'water': 0, 'closing': False},
	(36, 35): {'time': 140.44, 'water': 0, 'closing': False},
	(0, 30): {'time': 62.58, 'water': 0, 'closing': True},
	(30, 0): {'time': 51.42, 'water': 0, 'closing': True},
	(0, 37): {'time': 60.53, 'water': 0, 'closing': True},
	(37, 0): {'time': 53.47, 'water': 0, 'closing': True},
	(30, 37): {'time': 7.68, 'water': 0, 'closing': True},
	(37, 30): {'time': 6.25, 'water': 0, 'closing': True},
	(37, 38): {'time': 20.55, 'water': 0, 'closing': True},
	(38, 37): {'time': 17.45, 'water': 0, 'closing': True},
	(30, 32): {'time': 19.48, 'water': 0, 'closing': True},
	(32, 30): {'time': 18.52, 'water': 0, 'closing': True},
	(32, 38): {'time': 10.17, 'water': 0, 'closing': True},
	(38, 32): {'time': 8.4, 'water': 0, 'closing': True},
	(38, 39): {'time': 19.71, 'water': 0, 'closing': True},
	(39, 38): {'time': 18.29, 'water': 0, 'closing': True},
	(32, 34): {'time': 19.17, 'water': 0, 'closing': True},
	(34, 32): {'time': 18.83, 'water': 0, 'closing': True},
	(34, 39): {'time': 11.68, 'water': 0, 'closing': True},
	(39, 34): {'time': 11.53, 'water': 0, 'closing': True},
	(39, 40): {'time': 19.45, 'water': 0, 'closing': True},
	(40, 39): {'time': 18.55, 'water': 0, 'closing': True},
	(34, 36): {'time': 20.1, 'water': 0, 'closing': True},
	(36, 34): {'time': 17.9, 'water': 0, 'closing': True},
	(36, 40): {'time': 15.28, 'water': 0, 'closing': True},
	(40, 36): {'time': 12.57, 'water': 0, 'closing': True},
	(40, 10): {'time': 694.35, 'water': 0, 'closing': True},
	(10, 40): {'time': 656.44, 'water': 0, 'closing': True}
}

max_num_robots = 5
max_cleaning_time = 36000
water_capacity = 1000

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
