# ROS Services

Now that you've written your first ROS node, you've seen how publishing to a topic works, and you were able to control the robotic arm by publishing to the `/simple_arm/joint_1_position_controller/command` topic and `/simple_arm/joint_2_position_controller/command` topic. Next, we'll see another node called `arm_mover`, which implements the `safe_move` service to allow service calls to control the arm.

## Defining services

A ROS service allows **request/response** communication to exist between nodes. Within the node providing the service, request messages are handled by functions or methods. Once the requests have been handled successfully, the node providing the service sends a message back to the requester node. In C++, a ROS service server can be created using the following definition format:

```cpp
ros::ServiceServer service = n.advertiseService(`service_name`, handler);
```

In ROS, the service class name `ServiceServer` comes from the file name where the service definition exists. Each service provides a definition in a `.srv` file; this is a text file that provides the proper message type for both requests and responses.

The `advertiseService()` allows you to communicate with ROS through the node handle `n` and inform ROS that you want to create a service.

The `service_name` is the name given to the service. Other nodes will use this name to specify the service to which they are sending requests.

The `handler` is the name of the function or method that handles the incoming service message. This function is called each time the service is called, and the message from the service call is passed to the `handler` function as an argument. The `handler` should return an appropriate service response message.

## Using Services

- #### Command Line

Services can be called directly from the command line, with:

```sh
$ rosservice call service_name “request”
```

After calling the service, you will wait for an answer.

- #### ROS Service Client

Another approach is to use a ROS service programmatically, from within a node. You will define a `ROS client`, which provides the interface for sending messages to the service:

```C++
ros::ServiceClient client = n.serviceClient<package_name::service_file_name>("service_name");
```

One way the `ROS Client` can then be used is to send requests is as follows:

```C++
client.call(srv);    // request a service 
```

For now, we’ll focus on how to create the ROS **service server**. Later, in the `look_away` node, you will practice calling the service from a **service client** node.

See the ROS documentation [on services](http://wiki.ros.org/roscpp/Overview/Services) for detailed instructions on how to create and call ROS services.

![](/home/shengchen/gitbook/robotics_gitbook/ros/assets/screen-shot-2018-10-30-at-11.33.36-am.png)

## 