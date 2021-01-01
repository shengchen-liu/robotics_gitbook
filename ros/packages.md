# Packages & Catkin Workspaces

## Catkin packages

ROS software is organized and distributed into packages, which are directories that might contain source code for ROS nodes, libraries, datasets, and more.  Each package also contains a files with build instructions - the CMakeLists.txt file - and a package.xml file with information about the package.  Packages enable ROS users to organize useful functionality in a convenient and reusable format.

## Catkin Workspace

A catkin workspace is a top-level directory where you build, install, and modify catkin packages.  The workspace contains all the packages for your project, along with several other directories for the catking system to use when building executables and other targets from your source code.

### Create a Catkin Workspace

#### Step 1: Create a catkin workspace and a sub directory

All of the ROS-related code you develop throughout this course will reside in your catkin workspace. You only need to create and initialize the workspace once.

First, create the top level catkin workspace directory and a sub-directory named `src` (pronounced source). The top level directory name is arbitrary, but is often called `catkin_ws` (an abbreviation of catkin_workspace), so we will follow this convention. You can create these two directories in `/home/workspace/` with a single command:

```sh
$ mkdir -p /home/workspace/catkin_ws/src
```

#### Step 2: Navigate to the source directory

Next, navigate to the `src` directory with the cd command:

```sh
$ cd /home/workspace/catkin_ws/src
```

#### Step 3: Initialize the catkin workspace

Now you can initialize the catkin workspace which will create a `CMakeLists.txt` file:

```sh
$ catkin_init_workspace
```



[![img](assets/catkin-init-workspace.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/eba928a4-27ba-4e6a-a5f9-6a409da46b25/concepts/a5368ea8-3c59-49d4-b1ba-cd0ce6d2ee17#)



Let’s list the contents of the current directory to see what changed.

```sh
$ ls -l
```

Notice that a symbolic link (`CMakeLists.txt`) has been created to `/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake`

#### Step 4: Return to top level directory

Return to the catkin_ws,

```sh
$ cd /home/workspace/catkin_ws
```

#### Step 5: Build the Workspace

```sh
$ catkin_make
```

**Note**: you must issue this command from within the top level directory (i.e., within `catkin_ws` NOT `catkin_ws/src`)

While it is not essential that you have a deep understanding of the catkin build system, particularly if you are doing most of your development work in Python, it is helpful to learn about it. We encourage you to read the [ROS wiki](http://wiki.ros.org/catkin/conceptual_overview).

After the command is executed you will notice the output of the build processes being echoed to your display. When it has finished you should see the following lines at the end of the output:

```
-- BUILD_SHARED_LIBS is on
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/catkin_ws/build
#### Running command: "make -j4 -l4" in "/home/workspace/catkin_ws/build"
```

But what else has changed? Use the `ls` command again to see what is new.

```sh
$ ls
```



[![img](assets/ls.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/eba928a4-27ba-4e6a-a5f9-6a409da46b25/concepts/a5368ea8-3c59-49d4-b1ba-cd0ce6d2ee17#)



You now have two new directories: `build` and `devel`. The aptly named `build` directory is the build space for C++ packages and, for the most part, you will not interact with it. The `devel` directory does contain something of interest, a file named `setup.bash`. This setup.bash script must be sourced before using the catkin workspace with `source devel/setup.bash`

#### Optional

Before you begin to work with and develop your own ROS package, you can take a moment to get acquainted with catkin workspace conventional directory structure as described in the ROS Enhancement Proposal (REP) 128 by clicking [here](http://www.ros.org/reps/rep-0128.html).



[![img](assets/cmakelists.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/eba928a4-27ba-4e6a-a5f9-6a409da46b25/concepts/a5368ea8-3c59-49d4-b1ba-cd0ce6d2ee17#)

## Add a Package

### 1. Cloning the simple_arm Package

One of the biggest benefits of using ROS is that it has a really large community of users and developers who have already created a lot of code that you can reuse.

Let’s clone an existing package and add it to our newly created workspace.

Start by navigating to the `src` directory and cloning the `simple_arm` package for this lesson from its GitHub repo.

```sh
$ cd /home/workspace/catkin_ws/src/
$ git clone -b first_interaction https://github.com/udacity/RoboND-simple_arm/simple_arm
```

### 2. Building the simple_arm package

After the repo has finished cloning, you can change directory to the top-level of the ROS workspace and build the new package.

```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

## Rosdep

### ROS Package Dependencies

ROS packages have two types of dependencies: build dependencies and run dependencies.

The `rosdep` tool will check for a package's missing dependencies, download them, and install them.

To check for missing dependencies in a ROS package:

```sh
$ rosdep check <package name>
```

**Note**: In order for the command to work, the workspace must be first sourced with `source devel/setup.bash`.

This gives you a list of the system dependencies that are missing, and tells you where to get them.

To have `rosdep` install packages, invoke the following command from the root of the catkin workspace

```sh
$ rosdep install -i <package name>
```

### `simple_arm` Package Dependencies

Fortunately, the build and run dependencies for `simple_arm` package have been already installed in the workspace. You can double-check using `rosdep check`:

```sh
$ cd /home/workspace/catkin_ws/ 
$ source devel/setup.bash
$ rosdep check simple_arm
```

**Output:** All system dependencies have been satisfied.

## Create your own Packages

You'll begin your dive into ROS packages by creating one of your own. All ROS packages should reside under the `src` directory.

Assuming you have already sourced your ROS environment and your catkin workspace, navigate to the src directory:

```sh
$ cd /home/workspace/catkin_ws/src
```

The syntax for creating a catkin package is:

```sh
$ catkin_create_pkg <your_package_name> [dependency1 dependency2 …]
```

The name of your package is arbitrary but you will run into trouble if you have multiple packages with the same name in your catkin workspace. Try to make it descriptive and unique without being excessively long. Let’s name ours “first_package” and we won’t specify any dependencies. By convention, package names are lowercase.

```sh
$ catkin_create_pkg first_package
```

Voilà. You just created your first catkin package! Navigating inside our newly created package reveals that it contains just two files: `CMakeLists.txt` and `package.xml`. This is a minimum working catkin package. It is not very interesting because it doesn't do anything, but it meets all the requirements for a catkin package. One of the main functions of these two files is to describe dependencies and how catkin should interact with them. We won’t pay much attention to them right now.

I mentioned earlier that ROS packages have a conventional directory structure. Let’s take a look at a more typical package.

- scripts (python executables)
- src (C++ source files)
- msg (for custom message definitions)
- srv (for service message definitions)
- include -> headers/libraries that are needed as dependencies
- config -> configuration files
- launch -> provide a more automated way of starting nodes

Other folders may include

- urdf (Universal Robot Description Files)
- meshes (CAD files in .dae (Collada) or .stl (STereoLithography) format)
- worlds (XML like files that are used for Gazebo simulation environments)



[![img](assets/ros-package-format.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/eba928a4-27ba-4e6a-a5f9-6a409da46b25/concepts/5ce8e74a-b21f-414b-a5be-bf3c00c6f24c#)



There are many packages that you can install. To see a list of available packages for the Kinetic distribution, take some time to explore [the list of ROS package online](http://www.ros.org/browse/list.php).

