--------------------
Stating Shakey 2016:
--------------------

A. Create new (real) world:
	1.) Create new package (e.g. shakey_new_world)
	2.) Start mapping: roslaunch shakey_executable shakey_mapping.launch
	3.) Save map to shakey_new_world: rosrun map_server map_saver
	4.a) Create new folder ../src/shakey_planning_server/config/shakey_new_world
	4.b) Create Config files 
	4.c) For locations: if map is provided (rosrun shakey_utils geometryPosesCreatorGui map base_footprint) 
		Note: Add / before to /map in location file afterwards

B. Simulating is with gazebo (optional)
	- Start shakey in a world: roslaunch shakey_executable shakey_world pkg:='path to world pkg'
	- Insert Objects: roslaunch shakey_spawn_objects spawn_["box or wedge"].launch x:="x-value" y:="y-value" name:="obj_name"

C. Start Roboter system:
	1.) Start location: roslaunch shakey_executable shakey_localize.launch pkg:='path to world pkg' (here: ../src/shakey_new_world)
	2.) Start 2d-navigation: roslaunch shakey_2dnav shakey_2dnav.launch
	3.) Start object detection: roslaunch shakey_object_recognition shakey_object_recognition.launch
	4.) Start action server: roslaunch shakey_actionlib shakey_actionserver.launch
	5.) Start planner: roslaunch shakey_planning_server continual-planning-shakey.launch map_suffix:='suffiex of map' (here: shakey_new_world)

	Note: For 1.-5. You can use screenrun. (Example in shakey_two_world_room)

	Optional:
	6.) Start joystick/keyboard: roslaunch cmd_mux teleop_joystick.launch OR roslaunch cmd_mux teleop_keyboard.launch


---------------------------------- TODO: Remove this note ----------------------------------------------------------------------------------------

Record Rosbag:
	- rosbag record map particlecloud seg_cloud tf move_base/NavfnROS/plan move_base/TrajectoryPlannerROS/local_plan head_mount_kinect/rgb/image_raw head_mount_kinect/rgb/camera_info Detected_Objects Destination_Objects Objects_Location

Notes:
	- Start record before launching the planner (Object-Location Marker are only initialy published)
	- Set eval directories correctly
	- Save outputs with tee for Object-Detection?

