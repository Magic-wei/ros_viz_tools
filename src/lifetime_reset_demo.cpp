// Copyright (C) 2022 Wei Wang (wei.wang.bit@outlook.com)

#include "ros_viz_tools/ros_viz_tools.h"
#include <random>

static std::default_random_engine e;
std::uniform_int_distribution<int> randRGB(0, 255);

using ros_viz_tools::RosVizTools;
using ros_viz_tools::ColorMap;

int main( int argc, char** argv )
{
    ROS_INFO("lifetime_reset_demo starts.");
    ros::init(argc, argv, "lifetime_reset_demo");
    ros::NodeHandle n;
    ros::Publisher publisher1 = n.advertise<visualization_msgs::Marker>("text_reset", 1);
    ros::Publisher publisher2 = n.advertise<visualization_msgs::Marker>("text_reset_2", 1);
    ros::Publisher publisher3 = n.advertise<visualization_msgs::Marker>("text_to_delete", 1);
    std::string frame_id = "ros_viz_tools";
    std::string ns;
    geometry_msgs::Pose pose;
    double lifetime;

    ros::Rate r(1);

    bool publish_once = true;
    while (ros::ok())
    {
        // Publish once need to run this first
        r.sleep();

        // Reset marker
        ns = "text_reset";
        lifetime = 2.0;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 6.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker1 =
            RosVizTools::newText(1.0, pose, ns, 0, ros_viz_tools::WHITE, frame_id, lifetime);
        marker1.text = "This text marker (lifetime = 2.0) will be reset \nover time before lifetime is reached.";
        publisher1.publish(marker1);

        ns = "text_reset_2";
        lifetime = 0.5;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
        visualization_msgs::Marker marker2 =
            RosVizTools::newText(1.0, pose, ns, 0, ros_viz_tools::WHITE, frame_id, lifetime);
        marker2.text = "This text marker (lifetime = 0.5) will be \ncreated and deleted in each loop.";
        publisher2.publish(marker2);

        // Publish once
        if (publish_once) {
            publish_once = false;
            ns = "text_to_delete";
            lifetime = 2.0;
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 3.0;
            pose.orientation = tf2::toMsg(tf2::Quaternion(0 * M_PI / 180, 0 * M_PI / 180, 45 * M_PI / 180));
            visualization_msgs::Marker marker3 =
                RosVizTools::newText(1.0, pose, ns, 0, ros_viz_tools::WHITE, frame_id, lifetime);
            marker3.text = "This text marker (lifetime = 2.0) will be \ndeleted after 2 seconds.";
            publisher3.publish(marker3);
        }
    }
}