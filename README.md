# ros_viz_tools

This package is a visualization tool for easier Rviz [marker](http://wiki.ros.org/rviz/DisplayTypes/Marker) plotting.

## Dependencies

- Tested on Ubuntu 16.04 LTS & ROS kinetic
- tf2_geometry_msgs

## Quick Start

Build this package under your catkin workspace, run demo node and Rviz (frame: `ros_viz_tools`, topic: `demo_marker`) for a quick look:

```bash
# Clone this repo
cd <path_to_your_workspace>  # `cd ~/catkin_ws` for example
mkdir src && cd src
git clone git@github.com:Magic-wei/ros_viz_tools.git
# or - git clone https://github.com/Magic-wei/ros_viz_tools.git

# Build
cd <path_to_your_workspace>
catkin build ros_viz_tools
source devel/setup.bash

# Run regular demo
roslaunch ros_viz_tools demo_node.launch

# Run demo with lifetime setting
# - each marker will be auto-deleted once its lifetime is reached
roslaunch ros_viz_tools lifetime_demo.launch
# - three text markers are used to show how lifetime reset works
roslaunch ros_viz_tools lifetime_reset_demo.launch
```

![demo](./images/demo.png)

Currently support:

- Cube list
- Sphere list
- Line Strip
- Line List
- Cylinder
- Cube
- Sphere
- Arrow
- Text
- Frame (Axes-type Pose)

## Usage

Set catkin package dependencies in your `CMakeLists.txt` and `package.xml`,

```cmake
# CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  ...
  ros_viz_tools
)
```

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<package format="2">
  ...  
  <depend>ros_viz_tools</depend>
  ...
</package>
```

Include the header file in your codes,

```c++
#include "ros_viz_tools/ros_viz_tools.h"
```

### Create and Publish Markers

Initialize a `RosVizTools` instance named  `markers`,

```c++
ros::NodeHandle n;
std::string topic = "demo_marker";
ros_viz_tools::RosVizTools markers(n, topic);
```

Create a new marker and append it to `markers`. Let's take cube list marker for example.

```c++
// set marker frame id, namespace and id
std::string frame_id = "ros_viz_tools";
std::string ns = "cube_list";
int id = 0;
```

You can initialize a new marker by two approaches:

```c++
// intialize new marker by calling static member function in RosVizTools directly (recommended)
visualization_msgs::Marker marker = ros_viz_tools::RosVizTools::newCubeList(0.5, ns, id, ros_viz_tools::WHITE, frame_id);
// or by accessing the function through the instance
visualization_msgs::Marker marker = markers.newCubeList(0.5, ns, id, ros_viz_tools::WHITE, frame_id);
```

If the new marker involves a list (cube list, sphere list, line list or line strip), you also need to set a point list.

```c++
// modify marker, cube list, for example, also needs a point list.
for (int i = 0; i < 10; ++i) {
    geometry_msgs::Point p;
    p.x = i;
    p.y = pow(p.x, 2.0);
    p.z = 1.0;
    marker.points.push_back(p);
    std_msgs::ColorRGBA color = ros_viz_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
    marker.colors.push_back(color);
}
```

Append new marker to `RosVizTools` instance `markers`:

```c++
// append to markers
markers.append(marker);
```

At the end, call `publish()` function.

```c++
markers.publish();
```

Then you can open Rviz and see the markers published in the frame `ros_viz_tools` and topic `demo_marker`. Don't forget clear your markers at the beginning of every loop:

```c++
markers.clear();
```

Of course you can also publish your markers via your own publishers. For example, 

```cpp
// Publish single marker
ros::NodeHandle nh;
ros::Publisher publisher = nh.advertise<visualization_msgs::Marker>("your_topic_name", 1);
publisher.publish(marker);
```

You can see the following demo nodes for a better understanding of the usage for each marker type.

- [demo_node.cpp](./src/demo_node.cpp) - Create and publish all markers in a marker array, regular usage for most cases. This source file also contains example usage of colormap.
- [lifetime_demo.cpp](./src/lifetime_demo.cpp) - Publish all markers once and each marker will be deleted automatically once its lifetime is reached, except for the markers with the parameter `lifetime=0.0` which won't be auto-deleted, see [Lifetime](#Lifetime) section below for more details.
- [lifetime_reset_demo.cpp](./src/lifetime_reset_demo.cpp) - Three text markers are used to demonstrate how lifetime works for individual marker. This demo use three publishers for message type `visualization_msgs::Marker` to control these three markers instead of the built-in publisher in `ros_viz_tools::RosVizTools` class.

### Lifetime

The `lifetime` field of a marker specifies how long this marker should stick around before being automatically deleted. A value of `ros::Duration()` means never to auto-delete.

In class `ros_viz_tools::RosVizTools`, we use a double type parameter `lifetime` as the last parameter for all marker creator functions (`newMaker`, `newCubeList`, `newArrow`, etc.), which by default is 0.0 meaning never to auto-delete and any value greater than 0.0 will be treated as the lifetime in seconds of the marker.

```cpp
Marker RosVizTools::newMaker(const geometry_msgs::Vector3 &scale,
                             const geometry_msgs::Pose &pose,
                             const std::string &ns,
                             const int32_t &id,
                             const ColorRGBA &color,
                             const std::string &frame_id,
                             const int32_t &type,
                             const double &lifetime = 0.0) {
    // ...
    // Set lifetime
    if (lifetime == 0.0) {
        marker.lifetime = Marker::_lifetime_type(); // i.e. ros::Duration(), never to auto-delete
    } else {
        marker.lifetime = Marker::_lifetime_type(lifetime); // i.e. ros::Duration(t) with t in secs
    }
}
```

Note that if a new marker message with the same namespace and id is received before the lifetime has been reached, the lifetime will be reset to the value in the new marker message.

### Colors

To support colorful marker plotting, `ros_viz_tools` also defines functions and class for easier color settings. Now there are two approaches supported for generating colors:

* function `newColorRGBA` (also with some pre-defined colors in `color.h`) or
* class `ColorMap` (See `demo_node.cpp` for examples)

## License

This repository licensed under the [MIT License](./LICENSE).