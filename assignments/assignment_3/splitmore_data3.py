from splitmore import solve

groups = [
	{
		'Karim': 15.53,
		'Rasmus': -17.97,
		'Sarah': -37.39,
		'Alyssa': 39.83
	},
	{
		'Matthias': -3.64,
		'Lukas': -13.71,
		'Christian': -34.63,
		'Lisa': -23.93,
		'Christoph': 39.32,
		'Karim': 36.59
	},
	{
		'Franziska': -0.86,
		'Karim': -49.78,
		'Lisa': -43.99,
		'Luis': 26.96,
		'Lasse': 29.97,
		'Christian': 37.7
	},
	{
		'Christian': 8.93,
		'Karim': 22.08,
		'Christoph': -15.65,
		'Lukas': 46.21,
		'Hendrik': 12.24,
		'Sarah': -73.81
	},
	{
		'Lena': 21.11,
		'Anna': -1.19,
		'Lukas': -22.22,
		'Matthias': 2.3
	}
]

max_transactions_pp = 2

payment_methods = {
	'Karim': ['cash', 'banktransfer', 'splitmore'],
	'Rasmus': ['banktransfer'],
	'Sarah': ['banktransfer'],
	'Alyssa': ['splitmore', 'banktransfer'],
	'Matthias': ['splitmore'],
	'Lukas': ['banktransfer'],
	'Christian': ['splitmore', 'banktransfer', 'cash'],
	'Lisa': ['splitmore', 'banktransfer'],
	'Christoph': ['banktransfer', 'splitmore'],
	'Franziska': ['cash', 'splitmore'],
	'Luis': ['cash', 'banktransfer'],
	'Lasse': ['splitmore'],
	'Hendrik': ['cash', 'splitmore', 'banktransfer'],
	'Lena': ['banktransfer'],
	'Anna': ['cash', 'banktransfer', 'splitmore'],
}
solve(groups, max_transactions_pp, payment_methods)