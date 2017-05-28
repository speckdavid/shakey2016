# Shakey 2016
Shakey the robot was one of the first autonomous robots that showed impressive capabilities of navigation and mobile manipulation. The provided code is a today's (2016) implementation of Shakey with modern robotics technology. It is possible to run the Shakey 2016 project on a PR2 or as a simulation. More information in IEEE Robotics and Automation Letters (Vol. 2):  [Shakey 2016 — How Much Does it Take to Redo Shakey the Robot?](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7847341&isnumber=7797562)

[![Shakey 2016 - Video](images/youtube.png?raw=true)](https://www.youtube.com/watch?v=FQuuyhuEXak)

If you are intrested in trying out Shakey 2016, you can run and evaluate an example in Section 2. Shakey Quickscenario (with Simulation). If you want to create your own scenario either with simulation or with a real world PR2, you can find a detailed explanation in Section 3. Section 4 describes the execution of Shakey 2016. Finally, Section 5 is about evaluating recorded runs.
<br>

## 1. Installing Shakey 2016 <a name="installing"></a>
The Shakey 2016 system can be executed on a real robot, the PR2, or as a simulation. The Robot Operating System (ROS) is necessary to run this project. Our system is optimized (and recommended) for [ROS Hydro Medusa](http://wiki.ros.org/hydro/Installation/Ubuntu) and an Ubuntu 12.04 LTS (Precise) system. Once you have set up a [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) you can clone the Shakey 2016 Repository into your source folder (src) and build it as shown [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Now the Shakey 2016 project should be installed and is ready for use.

<br>

## 2. Shakey Quickscenario (with Simulation)
Here we explain how to run a provided scenario in order to test, reproduce and compare the system. First of all, start the simulation and the robot system.
```sh
$ roslaunch shakey_executable shakey_world.launch pkg:="$(rospack find shakey_quickscenario)"
```
You should tuck the arms of the robot such that they can not interact with the objects. For that purpose you can use teleop. The picture below illustrates the desired robot state.

```sh
$ roslaunch pr2_teleop_general pr2_teleop_general_keyboard.launch
```

![](images/pr2_arm_pose.jpg?raw=true)

If you want, you can start Rviz to visualize additional data of your run.
```sh
$ rosrun rviz rviz -d [...]/shakey_executable/shakey_rviz.rviz
```

Now start the Shakey 2016 system. In order to start a scenario we use [screenrun](http://wiki.ros.org/screenrun) with byobu. If you don't want to use byobu, you can use the optional instructions of section 2.1.
```sh
$ roslaunch shakey_quickscenario screenrun.launch
$ byobu
```
The planner window should appear which monitors the current state of the system.
Press "Execution/Run" to start the system. The robot will try to tidy up all objects located in the different rooms. Note: If you get an error message, wait a moment and try to press "Execution/Run" again (some modules may take some time to start).

![](images/planner.png?raw=true)

Now it is possible to evaluate and compare your run data. We provide a [python script](eval/) which creates plots of the run data ([here](shakey_quickscenario/eval/)). You can use the scipt as follows.
```sh
$ python3 eval/eval_data.py shakey_quickscenario/eval/[your_run_name (date)]
```

[Here](shakey_quickscenario/eval/example_plots) you can find some plots of a run we executed for comparison.

<br>

### 2.1 Optional: Start Shakey 2016 Modules by hand
If you don't want to use byobu, you can start all modules by hand as follows (use different tabs of your shell).
```sh
$ roslaunch shakey_executable shakey_localize.launch pkg:=$(rospack find shakey_quickscenario)
$ roslaunch shakey_2dnav shakey_2dnav.launch
$ roslaunch shakey_object_recognition shakey_object_recognition.launch
$ roslaunch shakey_actionlib shakey_actionserver.launch
$ roslaunch shakey_planning_server continual-planning-shakey.launch map_suffix:='shakey_quickscenario'
```

<br>

## 3. Create a new scenario (Simulation or Real World PR2)

In this section we explain how you can create your own scenario for the Shakey 2016 system. It is possible to use a simulation by building a map with gazebo. Or you can use a real world scenario with a real world PR2.

<br>

### 3.1 Start shakey_quickstart and rviz
Start the shakey quickstart tool to set up the new scenario.
```sh
$ rosrun shakey_quickstart shakey_quickstart
```
Start Rviz as an additional visualization tool.
```sh
$ rosrun rviz rviz -d [...]/shakey_executable/shakey_rviz.rviz
```

<br>

### 3.2 Create / Load new Scenario
Now you can create or load a new scenario. Inserting your descired scneario name and press Load /Create. If the a scenario with the desired name already exists, the relevant data will be loaded. Otherwise a new scenario with all relevant folders and files will be created.

![](images/scenario.jpg?raw=true)

<br>

### 3.3 Create a Map

Press "Start mapping" to build a map of the new scenario. The shakey_quickplay window should visualize the current map. Additionally, the map is published as "/map" which can be used to visualize it with Rviz. Once you are satisfied with the map you can press stop mapping and the map will be saved (indicaded by a small checkmark at the bottom). Note: Restarting the mapping procedure will remove the already created map.

![](images/mapping.jpg?raw=true)

<br>

### 3.4 Create relevant Poses
For a complete scenario it is neessary to specify Search Locations, Doorway Entries and Object Goal Locations. For that purpose you can use the Location area of shakey_quickstart (combined with rviz). At the top right you can choose between different topics which pusblish StampedPoses. We recommonend to use the "/move_base_simple/goal" topic. Furthermore, you can choose at bottom left which location type you want to specify next. In addition, you should choose the room in which the corresponding location should be located.

![](images/poses.jpg?raw=true)

In Rviz you can via drag and drop specify exact poses of the desired location. Select at top "2D Nav Goal" and click at the position you prefer. The already created locations are published as MarkerArray with topic "/poses_marker".

Search Locations are visualized as blue arrows, Doorways are visualized as red arrow with a connecting line between two corresponding entry points and Object Goal Locations as green boxes (irrelevant orientation).

![](images/rviz.jpg?raw=true)

<br>

### 3.5 Save the Scenario

Once you set all locations you can save them by pressing the Save button. If everything is set up you can press the Done! button and the shakey_quickstart window closes.

<br>

## 4. Start your Scenario

In order to start Shakey 2016 in your scenario follow the subsequent steps. If you use a real PR2 you can skip part 4.1.

<br>

### 4.1 Start the Simulation (optional)
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

### 4.2 Start Shakey 2016

In order to start a scenario we use [screenrun](http://wiki.ros.org/screenrun) with byobu.

```sh
$ roslaunch [your_scenario_name] screenrun.launch
$ byobu
```
You can navigate between different tabs with F3 and F4. Now, start all ros nodes by executing the provided commands (ENTER) in every tab. The planning monitor should appear and as explained in section 1 by pressing "Execution/Run" you can start the system.

<br>

## 5. Evaluate Runs
It is possible to evaluate and compare data of a single run or data of a collection of runs. The data of runs are stored in a subfolder eval/ located in the corresponding scenario folder. We provide a [python script](eval/) which creates plots of run data. The following command prints the usage of the script.
```sh
$ python3 eval/eval_data.py
```


<br>

## 6. Count Additional Code Lines
A script is provided [here](count_additional_code_lines/) to count the additional code lines (not published before) we had to write in order to create the Shakey 2016 system. The [Blacklist](count_additional_code_lines/black_list_files) and [Whitelist](count_additional_code_lines/white_list_dirs) show which files are counted (with comments).

<br>

## 7. References
[D. Speck, C. Dornhege and W. Burgard, Shakey 2016 — How Much Does it Take to Redo Shakey the Robot?, in IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 1203-1209, April 2017.](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7847341&isnumber=7797562)
