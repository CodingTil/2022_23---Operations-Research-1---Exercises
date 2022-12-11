from splitmore import solve

groups = [
	{
		'Lukas': -11.42,
		'Theresa': 48.85,
		'Anna': 7.41,
		'Lena': 0.46,
		'Christian': 42.98,
		'Matthias': -88.28
	},
	{
		'Karim': -22.78,
		'Marie': -24.11,
		'Hendrik': -20.57,
		'Lasse': -43.45,
		'Lukas': 110.91
	},
	{
		'Theresa': -13.34,
		'Clara': 27.3,
		'Amelie': 43.94,
		'Karim': -57.9
	}
]

max_transactions_pp = 3

payment_methods = {
	'Lukas': ['splitmore'],
	'Theresa': ['cash', 'splitmore'],
	'Anna': ['splitmore'],
	'Lena': ['splitmore'],
	'Christian': ['cash'],
	'Matthias': ['splitmore', 'banktransfer', 'cash'],
	'Karim': ['splitmore', 'cash', 'banktransfer'],
	'Marie': ['cash', 'splitmore'],
	'Hendrik': ['splitmore', 'banktransfer', 'cash'],
	'Lasse': ['banktransfer', 'cash', 'splitmore'],
	'Clara': ['cash', 'splitmore', 'banktransfer'],
	'Amelie': ['splitmore', 'banktransfer'],
}
solve(groups, max_transactions_pp, payment_methods)