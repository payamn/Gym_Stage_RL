import yaml

positions = open("pos.yaml", "r").read()
positions_array = []
for position in positions.split("---"):
	position_obj = yaml.load(position)
	if position_obj:
		positions_array.append(position_obj)

with open('positions.yaml', 'w') as outfile:
	dict_ = {}
	dict_['positions'] = positions_array
	yaml.dump(dict_, outfile, default_flow_style=False)
