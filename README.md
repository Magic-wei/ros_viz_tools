# ros_viz_tools

This package is a visualization tool for easier Rviz marker plotting.

## Dependencies

- Tested on Ubuntu 16.04 LTS & ROS kinetic
- tf2_geometry_msgs

## Quick Start

Build this package under your catkin workspace, run demo node and Rviz (frame: `ros_viz_tools`, topic: `demo_marker`) for a quick look:

```bash
rosrun ros_viz_tools demo_node
```

![demo](./images/demo.png)

Currently support:

- Cube list
- Sphere list
- Line Strip
- Line List
- Cylinder
- Cube
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

Initialize a `RosVizTools` instance named  `markers`,

```c++
ros::NodeHandle n;
std::string topic = "demo_marker";
RosVizTools markers(n, topic);
```

Create a new marker and append it to `markers`. Let's take cube list marker for example.

```c++
// set marker namespace and frame id
std::string ns = "cube_list";
std::string frame_id = "ros_viz_tools";
// intialize new marker
visualization_msgs::Marker marker = RosVizTools::newCubeList(0.5, ns, 0, WHITE, frame_id);
// modify marker, cube list, for example, also needs a position list.
for (int i = 0; i < 10; ++i) {
    geometry_msgs::Point p;
    p.x = i;
    p.y = pow(p.x, 2.0);
    p.z = 1.0;
    marker.points.push_back(p);
    std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
    marker.colors.push_back(color);
}
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

You can see [demo_node.cpp](./src/demo_node.cpp) for better understanding of the usage for each marker type.

## License

This repository licensed under the [MIT License](./LICENSE).