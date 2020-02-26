// Copyright (C) 2019 Wei Wang (wei.wang.bit@outlook.com)

#ifndef ROS_VIZ_TOOLS_H
#define ROS_VIZ_TOOLS_H

#include "ros_viz_tools/color.h"

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <string>

namespace ros_viz_tools {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using std_msgs::ColorRGBA;

class RosVizTools {
public:
    RosVizTools(const ros::NodeHandle &nh, const std::string &topic);

    void publish();

    void clear();

    void append(const Marker &marker);

    static Marker newMaker(const geometry_msgs::Vector3 &scale,
                           const geometry_msgs::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const ColorRGBA &color,
                           const std::string &frame_id,
                           const int32_t &type);

    static Marker newCubeList(double scale,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newSphereList(const double &scale,
                                const std::string &ns,
                                const int32_t &id,
                                const ColorRGBA &color,
                                const std::string &frame_id);

    static Marker newLineStrip(const double &scale,
                               const std::string &ns,
                               const int32_t &id,
                               const ColorRGBA &color,
                               const std::string &frame_id);

    static Marker newLineList(const double &scale,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newCylinder(const geometry_msgs::Vector3 &scale,
                              const geometry_msgs::Pose &pose,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newCube(const double &scale,
                          const geometry_msgs::Pose &pose,
                          const std::string &ns,
                          const int32_t &id,
                          const ColorRGBA &color,
                          const std::string &frame_id);

    static Marker newSphere(const double &scale,
                            const geometry_msgs::Pose &pose,
                            const std::string &ns,
                            const int32_t &id,
                            const ColorRGBA &color,
                            const std::string &frame_id);

    static Marker newArrow(const geometry_msgs::Vector3 &scale,
                           const geometry_msgs::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const ColorRGBA &color,
                           const std::string &frame_id);

    static Marker newText(const double &scale,
                          const geometry_msgs::Pose &pose,
                          const std::string &ns,
                          const int32_t &id,
                          const ColorRGBA &color,
                          const std::string &frame_id);

    static Marker newFrame(const double &width,
                           const double &length,
                           const geometry_msgs::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const std::string &frame_id);

    static geometry_msgs::Pose defaultPose();

private:
    void initPublisher();

private:
    ros::NodeHandle nh;
    ros::Publisher rviz_pub;
    std::string topic;
    MarkerArray rviz_marker_array;
};
} // namespace ros_viz_tools
#endif //ROS_VIZ_TOOLS_H
