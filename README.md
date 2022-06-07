# ROS 2

## Commands
`ros2 topic/service/action list -t` to show corresponded message/service/action types (what interface they are using) \
`ros2 interface show <type>` to introspect message/service/action types (.msg/.srv/action). Unlike ROS1 which uses rosmsg, ROS2 uses interface to show the types. \
underlay `source /opt/ros/foxy/setup.bash` \
overlay `source ws/install/setup.bash`

## Create workspace
```
mkdir -p ws/src
cd ws
colcon build
```

## Add existing package
in "/" of workspace
```
clone repo
rosdep install -i --from-path src --rosdistro <distro-version> -y     # foxy
colcon build   # in ws/
source install/setup.bash
```

## Create package
```
cd ws/src
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs
cd ..   # in ws/
colcon build    # always build in ws/
# colcon build --symlink-install     build with symbolic link so dont have to build for every change
source ws/install/setup.bash     # source pkg to use executables (ros2 run <pkg> <executable>)
```
Edit `<description>`, `<maintainer>` and `<license>` tags in package.xml \
Match them with `description`, `maintainer`, `maintainer_email` and `license` fields in setup.py \
Include dependency in package.xml with `<depend>` tag \
Add entry point in consolse_scripts of setup.py with `"<executable> = <pkg>.<file>:<function>"` so the node can be run with `ros2 run <pkg> <executable>` \
Always build the workspace and source again before ros run unless using symlink build

## Publisher and Subscriber
All nodes are inherited from rclpy.node and initialized with `super().__init("node_name")__` \
Could use timer together with publisher instead of while and sleep, but this will require `spin()` to trigger timer callback to publish to topics. \
Subscriber is similar to ROS1 and requires `spin()` to trigger callback.

## Service and Client
Service is similar to ROS1 that it requires `spin()` for callbacks. \
There are two types of clients in ROS2, synchronous clients and asynchronous clients. Synchronous clients are just like clients in ROS1 that block untill server send response. However, this can cause deadlock in ROS2. It is better to use asynchronous clients, but `spin()` is needed to trigger client update checking if response is ready.

## Create interface
Can only create interface in C++ packages right now. Could also write Python programs in C++ packages, but extra work is required to set it up.
```
cd ws/src
ros2 pkg create --build-type ament_cmake <interface_package>
cd <interface_package>
mkdir msg
mkdir srv
```
To convert the interface into language-specific code (C++ or Python), add the following to `CMakeLists.txt` before `ament_package()`
```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
)
```
Since the interfaces rely on `rosidl_default_generators` for generating language specific code, add the following to `package.xml` before `<export>` tag
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Now build the workspace again to use the created interface.

### Use custom interface
add `<deopend> interface_package </depend>` to `package.xml` and use the interface with `from interface_package.msg/srv import interface_filename`

After building the workspace, both ros2 run and python3 should be able to run nodes with custom interface. Susbect that this is due to interface import goes through rclpy (not 100% sure).


