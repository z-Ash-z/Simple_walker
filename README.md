# Simple_walker
A simple walker for the turtlebot. 

### Dependencies  
- Ubuntu 20.04 LTS/22.04 LTS
- ROS2 Humble
- colcon
- Gazebo
- rosdep
- turtlebot3

### Installing the turtlebot package
- Incase Turtlebot package is not installed use the follow command to install it
```
sudo apt install ros-humble-turtlebot3*
```

# Using the package

## Sourcing ROS2 Humble
- Based on installation of Humble, run:
```
. /opt/ros/humble/setup.bash
```
OR
```
. ~/ros2_humble/ros2-linux/setup.bash
```

## Building the package

### Clone the repository
- clone this repository into the `src` folder in your ros workspace.
```
cd <ros2 workspace folder>/src
git clone https://github.com/z-Ash-z/Simple_walker.git
cd ..
```

### Installing dependencies
- To install the dependencies this package requires run:
```
cd <ros2 workspace folder>
rosdep install -i --from-path src --rosdistro humble -y
```

### Building the package
- Using colcon to build the package and then sourcing the package.
```
colcon build --packages-select simple_walker && . install/setup.bash 
```

## Setting up the turtlebot
- Once the turtlebot package is installed, choose the turtlebot that you want to use:
```
export TURTLEBOT3_MODEL=burger
```

## Launching the turtlebot
- To launch the turtlebot in a pre-built world use:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Running the node
- After launching the turtlebot, in a new terminal run the `walker` node to control the turtlebot after sourcing ros and this package.
```
ros2 run simple_walker walker
```
- This node will control the trutlebot by using the data from `/scan` topic and publishing the command velocity to `/cmd_vel` topic.
- The turtlebot moves around the launched world while actively avoiding obstacles.

## Running the launch file
- Close all the previous terminals.
- In a new terminal, source ros2 and the package.
- Now use the launch file to launch the turtlebot and walker node.
```
ros2 launch simple_walker _walker_launch.py
```
- To record ros bags, use record_bag (type = bool) parameter. Usage `record_bag:=True`.
> To view all the available options run: ```ros2 launch pub_sub _pub_sub_custom_launch.py -s```

# Results

## Recording a ros bad
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the walker node or the launch file is running.
```
ros2 bag record --all -o package_output
```
- A sample of the results are stored in the [results/rosbag](/simple_walker/results/rosbag) folder. 

## Running CPP lint
- For style guide analysis, from the root directory of the project, run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/name,-build/include_order,-runtime/explicit --recursive simple_walker/. > simple_walker/results/cpplint.txt
```
- The results are in [cpplint.txt](/simple_walker/results/cpplint.txt)

## Running CPP check
- For static code analysis, from the root directory of the project run:
```
cppcheck --enable=all --std=c++17 simple_walker/src/ simple_walker/app/ simple_walker/include/simple_walker/ --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > simple_walker/results/cppcheck.txt
```
- The results are in [cppcheck.txt](/simple_walker/results/cppcheck.txt)