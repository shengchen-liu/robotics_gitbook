# Write ROS Nodes

## 1. ROS Publishers

Publishers allow a node to send messages to a topic, so that data from the node can be used in other parts of ROS.  In C++, ROS publishers typically have the following definition format, although other parameters and arguments are possible:

```cpp
ros::Publisher pub1 = n.advertise<message_type>("/topic_name", queue_size);
```

The `pub1` object is a publisher object instantiated from the ros::Publisher class. This object allows you to publish messages by calling the `publish()` function.

To communicate with ROS master in C++, you need a **NodeHandle**. The node handle `n` will fully initialize the node.

The `advertise()` function is used to communicate with ROS and inform that you want to publish a message on a given topic name. The `"/topic_name"` indicates which topic the publisher will be publishing to.

The message_type is the type of message being published on "/topic_name". For example, the string message data type in ROS is `std_msgs::String`.

The `queue_size` indicates the number of messages that can be stored in a queue. A publisher can store messages in a queue until the messages can be sent. If the number of messages stored exceeds the size of the queue, the oldest messages are dropped.

Once the publisher object `pub1` has been created, as above, a `message` with the specified data type can be published as follows:

```C++
pub1.publish(msg);
```

For more information about C++ ROS publishers, see [the documentation here](http://docs.ros.org/jade/api/roscpp/html/classros_1_1Publisher.html).

## 2. Node - Simple Mover

You will now go through the process of implementing your first ROS node in C++. This node is called `simple_mover`. As its name implies, this node only has one responsibility, and that is to command joint movements for `simple_arm`.

The goal of the `simple_mover` node is to command each joint in the simple arm and make it swing between -pi/2 to pi/2 over time. 

### Topics

To do so, it must publish joint angle command messages to the following topics:

| **Topic Name**   | /simple_arm/joint_1_position_controller/command              |
| :--------------- | :----------------------------------------------------------- |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 1 to move counter-clockwise, units in radians |

| **Topic Name**   | /simple_arm/joint_2_position_controller/command              |
| :--------------- | :----------------------------------------------------------- |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 2 to move counter-clockwise, units in radians |

### Build and Run

Before you can run the `simple_mover` node, you have to compile the C++ script.

### Modifying CMakeLists.txt

In order for catkin to generate the C++ libraries, you must first modify `simple_arm`’s `CMakeLists.txt`.

CMake is the build tool underlying catkin, and `CMakeLists.txt` is a CMake script used by catkin. If you’re familiar with the concept of makefiles, this is similar.

Navigate to the package `CMakeLists.txt` file and open it:

```sh
$ cd /home/shengchen/workspace/SimpleArm/catkin_ws/src/simple_arm/
$ gedit CMakeLists.txt 
```

First, ensure that the `find_package()` macro lists `std_msgs`, `message_generation`, and `controller_manager` as required packages. The `find_package()` macro should look as follows: