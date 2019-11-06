//
// Created by wangwei on 2019/11/4.
//

#include "ros_viz_tools/ros_viz_tools.h"
#include <random>

static std::default_random_engine e;
std::uniform_int_distribution<int> randRGB(0, 255);

int main( int argc, char** argv )
{
    ROS_INFO("demo_node starts.");
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle n;
    ros::NodeHandle private_n("~");
    ros::Rate r(1);
    std::string frame_id = "my_frame";
    std::string topic = "demo_marker";
    RosVizTools viz_tools(n, topic);
    std::string ns;
    while (ros::ok())
    {
        viz_tools.clearMakerArray();

        // Frame (Axes)
        visualization_msgs::Marker marker, marker2;
        ns = "axes";
        geometry_msgs::Pose pose;
        pose.position.x = 1.0;
        pose.position.y = 1.0;
        pose.position.z = 1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        marker = viz_tools.newFrame(0.1, 2.0, pose, ns, 0, BLUE, frame_id);
        pose.position.x = -1.0;
        pose.position.y = 2.0;
        pose.position.z = 3.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(30 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180));
        marker2 = viz_tools.newFrame(0.1, 1.0, pose, ns, 1, GREEN, frame_id);
        viz_tools.pushBackMarker(marker);
        viz_tools.pushBackMarker(marker2);

        // Cube List
        ns = "cube_list";
        visualization_msgs::Marker marker3 = viz_tools.newCubeList(0.5, ns, 0, WHITE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 1.0;
            marker3.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker3.colors.push_back(color);
        }
        viz_tools.pushBackMarker(marker3);

        // Line Strip
        ns = "line_strip";
        visualization_msgs::Marker marker4 = viz_tools.newLineStrip(0.3, ns, 0, LIGHT_BLUE, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 2.0;
            marker4.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker4.colors.push_back(color);
        }
        viz_tools.pushBackMarker(marker4);

        // Sphere List
        ns = "sphere_list";
        visualization_msgs::Marker marker5 = viz_tools.newSphereList(1.0, ns, 0, LIME_GREEN, frame_id);
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = pow(p.x, 2.0);
            p.z = 3.5;
            marker5.points.push_back(p);
            std_msgs::ColorRGBA color = newColorRGBA(randRGB(e), randRGB(e), randRGB(e));
            marker5.colors.push_back(color);
        }
        viz_tools.pushBackMarker(marker5);

        // Text
        ns = "text";
        pose.position.x = -2.0;
        pose.position.y = 2.0;
        pose.position.z = 2.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker6 = viz_tools.newText(1.0, pose, ns, 0, WHITE, frame_id);
        marker6.text = "This is text marker.";
        viz_tools.pushBackMarker(marker6);

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
        visualization_msgs::Marker marker7 = viz_tools.newCylinder(scale, pose , ns, 0, WHITE, frame_id);
        viz_tools.pushBackMarker(marker7);

        // Cube
        ns = "cube";
        pose.position.x = -1.0;
        pose.position.y = -1.0;
        pose.position.z = -1.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 45 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker8 = viz_tools.newCube(1.0, pose , ns, 0, WHITE, frame_id);
        viz_tools.pushBackMarker(marker8);

        // publish
        viz_tools.publish();

        r.sleep();
    }
}