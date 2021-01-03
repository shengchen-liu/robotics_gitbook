# Node - Simple Mover

You will now go through the process of implementing your first ROS node in C++. This node is called `simple_mover`. As its name implies, this node only has one responsibility, and that is to command joint movements for `simple_arm`.

The goal of the `simple_mover` node is to command each joint in the simple arm and make it swing between -pi/2 to pi/2 over time. 

## Topics

To do so, it must publish joint angle command messages to the following topics:

| **Topic Name**   | /simple_arm/joint_1_position_controller/command              |
| :--------------- | :----------------------------------------------------------- |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 1 to move counter-clockwise, units in radians |

| **Topic Name**   | /simple_arm/joint_2_position_controller/command              |
| :--------------- | :----------------------------------------------------------- |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 2 to move counter-clockwise, units in radians |

## Build and Run

Before you can run the `simple_mover` node, you have to compile the C++ script.

## Modifying CMakeLists.txt

In order for catkin to generate the C++ libraries, you must first modify `simple_arm`’s `CMakeLists.txt`.

CMake is the build tool underlying catkin, and `CMakeLists.txt` is a CMake script used by catkin. If you’re familiar with the concept of makefiles, this is similar.

Navigate to the package `CMakeLists.txt` file and open it:

```sh
$ cd /home/shengchen/workspace/SimpleArm/catkin_ws/src/simple_arm/
$ gedit CMakeLists.txt 
```

### a. find_package()

First, ensure that the `find_package()` macro lists `std_msgs`, `message_generation`, and `controller_manager` as required packages. The `find_package()` macro should look as follows:

```cmake
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
        controller_manager
)
```

As the names might imply, the `std_msgs` package contains all of the basic message types, and `message_generation` is required to generate message libraries for all the supported languages (cpp, lisp, python, javascript). The `contoller_manager` is another package responsible for controlling the arm.

### b. include directories, executable file, link libraries, and dependencies

Now, add the following block of code at the bottom of the file:

```cmake
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_mover src/simple_mover.cpp)
target_link_libraries(simple_mover ${catkin_LIBRARIES})
add_dependencies(simple_mover simple_arm_generate_messages_cpp)
```

These instructions ask the compiler to include the directories, executable file, link libraries, and dependencies for your C++ code:

```html
add_executable(node_name sourcecode_directory)
```

Creates the executable `simple_mover` file.

```html
target_link_libraries(node_name ${catkin_LIBRARIES})
```

This will add all the linked libraries to the compiler.

```html
add_dependencies(node_name package_name_generate_messages_cpp)
```

Generates message headers for this package before you can use them.

Keep in mind that you should always include these instructions whenever you want to write a C++ ROS node. For more information about `CMakeLists.txt` check out [the CMakeLists.txt page](http://wiki.ros.org/catkin/CMakeLists.txt) on the ROS wiki.

## Building the Package

Now that you have included specific instructions for your compiler, let’s build the package:

```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

## Running simple_mover

Asuming that your workspace has recently been built, you can launch `simple_arm` as follows:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch simple_arm robot_spawn.launch
```

Once the ROS Master, Gazebo, and all of our relevant nodes are up and running, we can finally launch `simple_mover`. To do so, open a new terminal and type the following commands:

```sh
$ cd ~/workspace/SimpleArm/catkin_ws
$ source devel/setup.bash
$ rosrun simple_arm simple_mover
```

## 