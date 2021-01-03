## ROS Publishers

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