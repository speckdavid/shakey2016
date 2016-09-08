# Shakey 2016
Shakey the robot was one of the first autonomous robots that showed impressive capabilities of navigation and mobile manipulation. The provided code is an today's (2016) implementation of Shakey with modern robotics technology. It is possible to run the Shakey 2016 project on a PR2 or as a simulation. 

## Installing Shakey 2016
The Shakey 2016 system can be executed on a real robot, the PR2, or as a simulation. The Robot Operating System (ROS) is necessary to run this project. Our system is optimized (and recommended) for [ROS Hydro Medusa](http://wiki.ros.org/hydro/Installation/Ubuntu) and an Ubuntu 12.04 LTS (Precise) system. Once you have set up a [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) you can clone the Shakey 2016 Repository into your source folder (src) and build it as shown [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Now the Shakey 2016 project should be installed and ready for use.


## Stating Shakey 2016:
1. Create new (real) world:
  1. Create new package (e.g. shakey_new_world)
  2. Start mapping: roslaunch shakey_executable shakey_mapping.launch
  3. Save map to shakey_new_world: rosrun map_server map_saver
  4. Create new folder ../src/shakey_planning_server/config/shakey_new_world
  5. Create Config files (you can copy the configs from [shakey_two_room_world](shakey_two_room_world/)
  6. For locations in locations.dat: if map is provided (rosrun shakey_utils geometryPosesCreatorGui map base_footprint) - Note: Add / before to /map in location file afterwards

2. Simulating is with gazebo (optional)
  * Start shakey in a world: roslaunch shakey_executable shakey_world pkg:='path to world pkg'
  * Insert Objects: roslaunch shakey_spawn_objects spawn_["box or wedge"].launch x:="x-value" y:="y-value" name:="obj_name"

3. Start Roboter system:
  1. Start location: roslaunch shakey_executable shakey_localize.launch pkg:='path to world pkg' (here: ../src/shakey_new_world)
  2. Start 2d-navigation: roslaunch shakey_2dnav shakey_2dnav.launch
  3. Start object detection: roslaunch shakey_object_recognition shakey_object_recognition.launch
  4. Start action server: roslaunch shakey_actionlib shakey_actionserver.launch
  5. Start planner: roslaunch shakey_planning_server continual-planning-shakey.launch map_suffix:='suffiex of map' (here: shakey_new_world)
  6. Move the arms of the robot in a tucked position. You can use roslaunch cmd_mux teleop_joystick.launch OR roslaunch cmd_mux teleop_keyboard.launch for it.
  7. Note: For 1.-5. You can use [screenrun](http://wiki.ros.org/screenrun) with byobu. (Example in shakey_two_world_room)

Of course you can also use the provided worlds shakey_two_rooms_world and shakey_three_rooms_world.


