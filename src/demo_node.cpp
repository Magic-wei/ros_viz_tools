// Copyright (C) 2019 Wei Wang (wei.wang.bit@outlook.com)

#include "ros_viz_tools/ros_viz_tools.h"
#include <random>

static std::default_random_engine e;
std::uniform_int_distribution<int> randRGB(0, 255);

int main( int argc, char** argv )
{
    ROS_INFO("demo_node starts.");
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle n;
    std::string topic = "demo_marker";
    RosVizTools markers(n, topic);
    std::string frame_id = "ros_viz_tools";
    std::string ns;

    ros::Rate r(1);
    while (ros::ok())
    {
        markers.clear();

        // Frame (Axes)
        visualization_msgs::Marker marker, marker2;
        ns = "axes";
        geometry_msgs::Pose pose;
        pose.position.x = -1.0;
        pose.position.y = 1.0;
        pose.position.z = -1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        marker = RosVizTools::newFrame(0.1, 2.0, pose, ns, 0, BLUE, frame_id);
        pose.position.x = -1.0;
        pose.position.y = 2.0;
        pose.position.z = -3.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(30 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180));
        marker2 = RosVizTools::newFrame(0.1, 1.0, pose, ns, 1, GREEN, frame_id);
        markers.append(marker);
        markers.append(marker2);

        // Cube List
        ns = "cube_list";
        visualization_msgs::Marker marker3 = RosVizTools::newCubeList(0.5, ns, 0, WHITE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 1.0;
            marker3.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker3.colors.push_back(color);
        }
        markers.append(marker3);

        // Line Strip
        ns = "line_strip";
        visualization_msgs::Marker marker4 = RosVizTools::newLineStrip(0.3, ns, 0, LIGHT_BLUE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 2.0;
            marker4.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker4.colors.push_back(color);
        }
        markers.append(marker4);

        // Sphere List
        ns = "sphere_list";
        visualization_msgs::Marker marker5 = RosVizTools::newSphereList(1.0, ns, 0, LIME_GREEN, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 3.5;
            marker5.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker5.colors.push_back(color);
        }
        markers.append(marker5);

        // Text
        ns = "text";
        pose.position.x = -2.0;
        pose.position.y = 2.0;
        pose.position.z = 2.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker6 = RosVizTools::newText(1.0, pose, ns, 0, WHITE, frame_id);
        marker6.text = "This is text marker.";
        markers.append(marker6);

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
        visualization_msgs::Marker marker7 = RosVizTools::newCylinder(scale, pose , ns, 0, WHITE, frame_id);
        markers.append(marker7);

        // Cube
        ns = "cube";
        pose.position.x = -1.0;
        pose.position.y = -1.0;
        pose.position.z = -1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker8 = RosVizTools::newCube(1.0, pose , ns, 0, WHITE, frame_id);
        markers.append(marker8);

        // publish
        markers.publish();

        r.sleep();
    }
}