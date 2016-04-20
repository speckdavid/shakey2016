Reimplementation of Shakey:

Create new (real) world:
	1.) Create new package (e.g. shakey_new_world)
	2.) Start mapping: roslaunch shakey_executable shakey_mapping.launch
	3.) Save map to shakey_new_world: rosrun map_server map_saver
	4.a) Create new folder ../src/shakey_planning_server/config/shakey_new_world
	4.b) Create Config files

Start Roboter system:
	1.) Start location: roslaunch shakey_executable shakey_localize.launch pkg:='path to world pkg' (here: ../src/shakey_new_world)
	2.) Start 2d-navigation: roslaunch shakey_2dnav shakey_2dnav.launch
	3.) Start object detection: roslaunch shakey_object_recognition shakey_object_recognition.launch
	4.) Start action server: roslaunch shakey_actionlib shakey_actionserver.launch
	5.) Start planner: roslaunch shakey_planning_server continual-planning-shakey.launch map_suffix:='suffiex of map' (here: shakey_new_world)
	6.a) Start cmd_mux: roslaunch cmd_mux cmd_mux.launch

	Note: For 1.-6.a) You can use screenrun. (Example in shakey_original world)

	Optional:
	6.b) Start joystick/keyboard: roslaunch cmd_mux teleop_joystick.launch OR roslaunch cmd_mux teleop_keyboard.launch


Note: Simulating is possible with gazebo
	- Start shakey in a world: roslaunch shakey_executable shakey_world pkg:='path to world pkg'
	- Insert Objects: roslaunch shakey_spawn_objects spawn_["box or wedge"].launch x:="x-value" y:="y-value"

Record Rosbag: [map, particle, global plan, local plan, object marker, object destination marker]
	- rosbag record map particlecloud move_base/TrajectoryPlannerROS/global_plan move_base/TrajectoryPlannerROS/local_plan Detected_Objects Destination_Objects

Additional: [head mount image, segmentation cloud]
	- head_mount_kinect/rgb/image_raw seg_cloud

