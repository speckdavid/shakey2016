Reimplementation of Shakey:

Start shakey in a world:
	roslaunch shakey_executable shakey_world pkg:='name of world pkg'

Start mapping:
	roslaunch shakey_executable shakey_mapping.launch

Start location:
	roslaunch shakey_executable shakey_localize.launch pkg:='name of world pkg'

Start 2d-navigation (location must run):
	roslaunch shakey_2dnav shakey_2dnav.launch

