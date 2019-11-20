// Copyright (C) 2019 Wei Wang (wei.wang.bit@outlook.com)

#ifndef ROS_VIZ_TOOLS_H
#define ROS_VIZ_TOOLS_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <string>

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using std_msgs::ColorRGBA;

ColorRGBA newColorRGBA(uint8_t red, uint8_t green, uint8_t blue, double alpha = 1.0);

// pre-defined color
const ColorRGBA WHITE      = newColorRGBA(255, 255, 255);
const ColorRGBA BLACK      = newColorRGBA(  0,   0,   0);
const ColorRGBA RED        = newColorRGBA(255,   0,   0);
const ColorRGBA GREEN      = newColorRGBA(  0, 255,   0);
const ColorRGBA BLUE       = newColorRGBA(  0,   0, 255);
const ColorRGBA YELLOW     = newColorRGBA(255, 255,   0);
const ColorRGBA CYAN       = newColorRGBA(  0, 255, 255);
const ColorRGBA MAGENTA    = newColorRGBA(255,   0, 255);
const ColorRGBA GRAY       = newColorRGBA(128, 128, 128);
const ColorRGBA PURPLE     = newColorRGBA(128,   0, 128);
const ColorRGBA PINK       = newColorRGBA(255, 192, 203);
const ColorRGBA LIGHT_BLUE = newColorRGBA(173, 216, 230);
const ColorRGBA LIME_GREEN = newColorRGBA( 50, 205,  50);
const ColorRGBA SLATE_GRAY = newColorRGBA(112, 128, 144);

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
                           const std_msgs::ColorRGBA &color,
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

#endif //ROS_VIZ_TOOLS_H
