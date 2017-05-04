# Shakey 2016
Shakey the robot was one of the first autonomous robots that showed impressive capabilities of navigation and mobile manipulation. The provided code is a today's (2016) implementation of Shakey with modern robotics technology. It is possible to run the Shakey 2016 project on a PR2 or as a simulation. More information in IEEE Robotics and Automation Letters (Vol. 2):  [Shakey 2016 — How Much Does it Take to Redo Shakey the Robot?](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7847341&isnumber=7797562)

[![Shakey 2016 - Video](images/youtube.png?raw=true)](https://www.youtube.com/watch?v=wKOCdZT4GXM&feature=youtu.be)

<br>

## 0. Installing Shakey 2016 <a name="installing"></a>
The Shakey 2016 system can be executed on a real robot, the PR2, or as a simulation. The Robot Operating System (ROS) is necessary to run this project. Our system is optimized (and recommended) for [ROS Hydro Medusa](http://wiki.ros.org/hydro/Installation/Ubuntu) and an Ubuntu 12.04 LTS (Precise) system. Once you have set up a [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) you can clone the Shakey 2016 Repository into your source folder (src) and build it as shown [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Now the Shakey 2016 project should be installed and is ready for use.

<br>

## 1. Shakey Quickstart (with Simulation)
Here we explain how to run an already existing scenario. First of all, start the simulation and the robot system.
```sh
$ roslaunch shakey_executbale shakey_world.launch pkg:="$(rospack find shakey_quickscenario)" &
```
You can (and should) tuck the arms of the robot such that they not infer with the objects. For that purpose you can use teleop.

```sh
$ roslaunch pr2_teleop_general pr2_teleop_general_keyboard.launch
```

Now start the Shakey 2016 system. In order to start a scenario we use [screenrun](http://wiki.ros.org/screenrun) with byobu.
```sh
$ roslaunch shakey_quickscenario screenrun.launch
$ byobu
```
The planner window should appear which monitors the current state of the system.
By pressing "Execution/Run" the system will start to tidy up the objects located in the different rooms.

![](images/planner.png?raw=true)

<br>

## 2. Create new scenario
### 2.1 Start shakey_quickstart and rviz
Start the shakey quickstart tool to set up the new scenario.
```sh
$ rosrun shakey_quickstart shakey_quickstart
```
Start Rviz as an additional visualization tool.
```sh
$ rosrun rviz rviz
```

<br>

### 2.2 Create / Load new Scenario
Now you can create or load a new scenario. Inserting your descired scneario name and press Load /Create. If the a scenario with the desired name already exists, the relevant data will be loaded. Otherwise a new scenario with all relevant folders and files will be created.

![](images/scenario.jpg?raw=true)

<br>

### 2.3 Create a Map

Press "Start mapping" to build up a map of the new scenario. The shakey_quickplay window should visualize the current map. Additionally, the map is published as "/map" which can be used to visualize it with Rviz. Once you are satified with the map you can press stop mapping and the map will be saved (indicaded by a small checkmark at the bottom). Note: Restarting the mapping procedure will remove the already created map.

![](images/mapping.jpg?raw=true)

<br>

### 2.4 Create relevant Poses
For a complete scenario it is neessary to specify Search Locations, Doorway Entries and Object Goal Locations. For that purpose you can use the Location area of shakey_quickstart (combined with rviz). At the top right you can choose between different topics which pusblish StampedPoses. We recommonend to use the "/move_base_simple/goal" topic. Furthermore, you can choose at bottom left which location type you want to specify next. In addition, you should choose the room in which the corresponding location is located.

![](images/poses.jpg?raw=true)

Via drag and drop you can specify the exact poses of the desired location. Choose at he ttop "2D Nav Goal" and select the pose you prefer. The already created locations are published as "/poses_marker" and can be visualized as MarkerArray in Rviz.

Search Locations are visualized as blue arrows, Doorways are visualized as red arrow with a connecting line between two corresponding entry points and Object Goal Locations as green boxes (irrelevant orientation).

![](images/rviz.jpg?raw=true)

<br>

### 2.5 Save the Scenario

Once you set all locations you can save them by pressing the Save button. If everything is set up you can press the Done! button and the shakey_quickstart window closes.

<br>

## 3. Start a Scenario

### 3.1 Start the Simulation (optional)
If you work with a "real" PR2 you can skip this part. Otherwise start the simualtion with gazebo.

```sh
$ roslaunch shakey_executable shakey_world.launch pkg:="$(rospack find [your_scenario_name])"
```

You can insert objects (boxes and wedges) with the following command.
```sh
$ roslaunch shakey_spawn_objects spawn_["box or wedge"].launch x:="x-value" y:="y-value" name:="[obj_name]"
```
E.g. a box at the origin with name box_0 can be created as follows.
```sh
$ roslaunch shakey_spawn_objects spawn_box.launch x:="0" y:="0" name:="box_0"
```

<br>

### 3.2 Start Shakey 2016

In order to start a scenario we use [screenrun](http://wiki.ros.org/screenrun) with byobu.

```sh
$ roslaunch [your_scenario_name] screenrun.launch
$ byobu
```
You can navigate between different taps with F3 and F4. Now, start all ros nodes by executing the provided commands (ENTER) in every tap. The planning monitor should appear and as explained in 1. by pressing "Execution/Run" you can start the system.

<br>

## 4. Evaluate Runs
 Work in procress...

<br>

## 5. Count Additional Code Lines
A script is provided [here](count_additional_code_lines/) to count the additional code lines (not published before) we had to write in order to create the Shakey 2016 system. The [Blacklist](count_additional_code_lines/black_list_files) and [Whitelist](count_additional_code_lines/white_list_dirs) show which files are counted (with comments).

<br>

## 6. References
[D. Speck, C. Dornhege and W. Burgard, Shakey 2016 — How Much Does it Take to Redo Shakey the Robot?, in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 1203-1209, April 2017.](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7847341&isnumber=7797562)
