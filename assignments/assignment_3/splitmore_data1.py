from splitmore import solve

groups = [
	{
		'Emilio': 36.4,
		'Christian': -36.4
	},
	{
		'Christoph': 4.75,
		'Sarah': -4.75
	},
	{
		'Sarah': 49.45,
		'Hannah': 46.08,
		'Luis': -95.53
	}
]

max_transactions_pp = 2

payment_methods = {
	'Emilio': ['splitmore'],
	'Christian': ['splitmore', 'banktransfer', 'cash'],
	'Christoph': ['banktransfer', 'splitmore'],
	'Sarah': ['banktransfer', 'splitmore', 'cash'],
	'Hannah': ['splitmore', 'banktransfer'],
	'Luis': ['banktransfer'],
}
solve(groups, max_transactions_pp, payment_methods)