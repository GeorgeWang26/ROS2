# ROS 2

## Commands
Unlike ROS1 which uses rosmsg, ROS2 uses interface to show the types. \
`ros2 topic/service/action list -t` to show corresponded message/service/action types (what interface they are using) \
`ros2 interface show <type>` to introspect message/service/action types (.msg/.srv/.action).

underlay `source /opt/ros/foxy/setup.bash` \
overlay `source ws/install/setup.bash` \

If `colcon build` raise error "Could not find a package configuration file provided by xxx" just install the missing package with `sudo apt-get install ros-<ros_version>-xxx`

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
Edit `<description>`, `<maintainer>` and `<license>` tags in package.xml and match them with `description`, `maintainer`, `maintainer_email` and `license` fields in setup.py \
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

## Interface
Can only create interface in C++ packages right now. Could also write Python programs in C++ packages, but extra work is required to set it up.
```
cd ws/src
ros2 pkg create --build-type ament_cmake <interface_package>
cd <interface_package>
mkdir msg
mkdir srv
mkdir action
```
To convert the interface into language-specific code (C++ or Python), add the following to `CMakeLists.txt` before `ament_package()`
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
  "action/Fibonacci.action"
)
```
Since the interfaces rely on `rosidl_default_generators` for generating language specific code, add the following to `package.xml` before `<export>` tag
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<!-- only needed when creating action interface -->
<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
Now build the workspace again to use the created interface.

### Use custom interface
add `<deopend> interface_package </depend>` to `package.xml` and use the interface with
```
from interface_package.msg import Num
from interface_package.srv import AddThreeInts
from interface_package.action import Fibonacci
```
After building the workspace, both ros2 run and python3 should be able to run nodes with custom interface. Susbect that this is due to interface import goes through rclpy (not 100% sure).

## Parameters
Unlike ROS1 where parameters all exist under ros master, in ROS2 parameters exist under each node. Since there are no ros master in ROS2, direct access to paramters under other nodes is not possible. Use a service on the param node as getter and setter for the parameter, so other nodes can interact with this param. Because all params exist under node, there are no more pathing as in ROS1, just use `path/subpath1/subpath2`

## Action Server and Client
Just like services, client send request to server, server send a result back to client. However, server can also send **optional** feedback to client while processing the request, before response is sent out. \
`spin()` is needed for both server and client. Server need to activate callback to process request, and client need to use callback to get feedback and result because requests are async.

## Launch
`ros2 launch <package_name> <launch_file_name>` \
Add `<exec_depend>ros2launch</exec_depend>` in package.xml to make sure `ros2 launch` is available after building the workspace \

In order for colcon to find the launch files, update the `data_files` parameter in `setup.py`
```
from setuptools import setup
import os
from glob import glob

package_name = "package_name"

setup(
    # other parameters
    data_files=[
        # other data files
        # now all launch files with .launch.py suffix will be runnable
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ]
)
```

Sample launch file
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='examples',
            executable='param',
            name='custom_parameter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
```
