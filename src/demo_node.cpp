// Copyright (C) 2019 Wei Wang (wei.wang.bit@outlook.com)

#include "ros_viz_tools/ros_viz_tools.h"
#include <random>

static std::default_random_engine e;
std::uniform_int_distribution<int> randRGB(0, 255);

using ros_viz_tools::RosVizTools;
using ros_viz_tools::ColorMap;

int main( int argc, char** argv )
{
    ROS_INFO("demo_node starts.");
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle n;
    std::string topic = "demo_marker";
    ros_viz_tools::RosVizTools markers(n, topic);
    std::string frame_id = "ros_viz_tools";
    std::string ns;

    ros::Rate r(1);
    while (ros::ok())
    {
        markers.clear();

        // Frame (Axes)
        visualization_msgs::Marker marker_frame1, marker_frame2;
        ns = "axes";
        geometry_msgs::Pose pose;
        pose.position.x = -3.0;
        pose.position.y = 1.0;
        pose.position.z = -1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        marker_frame1 = RosVizTools::newFrame(0.1, 2.0, pose, ns, 0, frame_id);
        pose.position.x = -5.0;
        pose.position.y = 2.0;
        pose.position.z = -3.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(30 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180));
        marker_frame2 = RosVizTools::newFrame(0.1, 1.0, pose, ns, 1, frame_id);
        markers.append(marker_frame1);
        markers.append(marker_frame2);

        // Cube List
        ns = "cube_list";
        visualization_msgs::Marker marker_cubelist = RosVizTools::newCubeList(0.5, ns, 0, ros_viz_tools::WHITE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 1.0;
            marker_cubelist.points.push_back(p);
            std_msgs::ColorRGBA color = ros_viz_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker_cubelist.colors.push_back(color);
        }
        markers.append(marker_cubelist);

        // Line Strip
        ns = "line_strip";
        visualization_msgs::Marker marker_linestrip = RosVizTools::newLineStrip(0.3, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 2.0;
            marker_linestrip.points.push_back(p);
            std_msgs::ColorRGBA color = ros_viz_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker_linestrip.colors.push_back(color);
        }
        markers.append(marker_linestrip);

        // Sphere List
        ns = "sphere_list";
        visualization_msgs::Marker marker_spherelist = RosVizTools::newSphereList(1.0, ns, 0, ros_viz_tools::LIME_GREEN, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 3.5;
            marker_spherelist.points.push_back(p);
            std_msgs::ColorRGBA color = ros_viz_tools::newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker_spherelist.colors.push_back(color);
        }
        markers.append(marker_spherelist);

        // Text
        ns = "text";
        pose.position.x = -2.0;
        pose.position.y = 2.0;
        pose.position.z = 2.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker_text = RosVizTools::newText(1.0, pose, ns, 0, ros_viz_tools::WHITE, frame_id);
        marker_text.text = "This is text marker.";
        markers.append(marker_text);

        // Cylinder
        ns = "cylinder";
        geometry_msgs::Vector3 scale;
        scale.x = 0.5;
        scale.y = 0.5;
        scale.z = 1.0;
        pose.position.x = -2.0;
        pose.position.y = -2.0;
        pose.position.z = -2.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker_cylinder = RosVizTools::newCylinder(scale, pose , ns, 0, ros_viz_tools::WHITE, frame_id);
        markers.append(marker_cylinder);

        // Cube
        ns = "cube";
        pose.position.x = -1.0;
        pose.position.y = -1.0;
        pose.position.z = -1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker_cube = RosVizTools::newCube(1.0, pose , ns, 0, ros_viz_tools::WHITE, frame_id);
        markers.append(marker_cube);

        // Cube
        ns = "sphere";
        pose.position.x = -3.0;
        pose.position.y = -3.0;
        pose.position.z = -3.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker_sphere = RosVizTools::newSphere(0.5, pose , ns, 0, ros_viz_tools::RED, frame_id);
        markers.append(marker_sphere);

        // Arrow
        ns = "arrow";
        scale.x = 1.0;
        scale.y = 0.1;
        scale.z = 0.1;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 90 * M_PI / 180));
        visualization_msgs::Marker marker_arrow = RosVizTools::newArrow(scale, pose , ns, 0, ros_viz_tools::WHITE, frame_id);
        markers.append(marker_arrow);

        // ColorMap
        size_t start_color = ColorMap::colorRGB2Hex(255, 0, 0);
        size_t end_color = ColorMap::colorRGB2Hex(0, 255, 0);
        std::vector<size_t> color_list1, color_list2;
        ColorMap::linspaceColorRGBinHex(start_color, end_color, 2, color_list1);
        ColorMap colormap1(color_list1);
        color_list2.push_back(ColorMap::colorRGB2Hex(255, 0, 0));
        color_list2.push_back(ColorMap::colorRGB2Hex(0, 255, 0));
        ColorMap colormap2(color_list2);
        ns = "colormap";
        visualization_msgs::Marker marker_colormap1 = RosVizTools::newLineStrip(0.2, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
        visualization_msgs::Marker marker_colormap2 = RosVizTools::newSphereList(0.2, ns, 1, ros_viz_tools::LIGHT_BLUE, frame_id);
        for (int i = 0; i < 20; ++i) {
            geometry_msgs::Point p;
            p.x = i * 0.5;
            p.y = sin(p.x);
            p.z = -1.0;
            auto scale_value = (p.y + 1.0) / 2.0;
            marker_colormap1.points.push_back(p);
            uint8_t red, green, blue;
            ColorMap::colorHex2RGB(colormap1(scale_value), red, green, blue);
            std_msgs::ColorRGBA color = ros_viz_tools::newColorRGBA(red, green, blue);
            marker_colormap1.colors.push_back(color);
            p.z = -2.0;
            marker_colormap2.points.push_back(p);
            ColorMap::colorHex2RGB(colormap2(scale_value), red, green, blue);
            color = ros_viz_tools::newColorRGBA(red, green, blue);
            marker_colormap2.colors.push_back(color);

        }
        markers.append(marker_colormap1);
        markers.append(marker_colormap2);

        // publish
        markers.publish();

        r.sleep();
    }
}