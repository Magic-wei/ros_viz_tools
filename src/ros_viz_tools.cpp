// Copyright (C) 2019 Wei Wang (wei.wang.bit@outlook.com)

#include "ros_viz_tools/ros_viz_tools.h"
namespace ros_viz_tools {
RosVizTools::RosVizTools(const ros::NodeHandle &nh, const std::string &topic) :
        nh(nh), topic(topic) {
    initPublisher();
}

void RosVizTools::initPublisher() {
    rviz_pub = this->nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

void RosVizTools::publish() {
    rviz_pub.publish(this->rviz_marker_array);
}

void RosVizTools::clear() {
    this->rviz_marker_array.markers.clear();
}

void RosVizTools::append(const Marker &marker) {
    this->rviz_marker_array.markers.push_back(marker);
}

Marker RosVizTools::newMaker(const geometry_msgs::Vector3 &scale,
                             const geometry_msgs::Pose &pose,
                             const std::string &ns,
                             const int32_t &id,
                             const ColorRGBA &color,
                             const std::string &frame_id,
                             const int32_t &type,
                             const double &lifetime) {
    Marker marker;
    // Set marker frame and timestamp.
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.
    marker.ns = ns;
    marker.id = id;

    // Set the marker type.
    marker.type = type;

    // Set the marker action.
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    marker.pose = pose;

    // Set the scale and color of the marker.
    marker.scale = scale;
    marker.color = color;

    // Set lifetime
    if (lifetime == 0.0) {
        marker.lifetime = Marker::_lifetime_type(); // ros::Duration(), never to auto-delete
    } else {
        marker.lifetime = Marker::_lifetime_type(lifetime); // ros::Duration(t) with t in secs
    }

    return marker;
}

Marker RosVizTools::newCubeList(double scale,
                                const std::string &ns,
                                const int32_t &id,
                                const ColorRGBA &color,
                                const std::string &frame_id,
                                const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = scale;
    vec_scale.z = scale;
    return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id, visualization_msgs::Marker::CUBE_LIST, lifetime);
}

Marker RosVizTools::newSphereList(const double &scale,
                                  const std::string &ns,
                                  const int32_t &id,
                                  const ColorRGBA &color,
                                  const std::string &frame_id,
                                  const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = scale;
    vec_scale.z = scale;
    return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id, visualization_msgs::Marker::SPHERE_LIST, lifetime);
}

Marker RosVizTools::newLineStrip(const double &scale,
                                 const std::string &ns,
                                 const int32_t &id,
                                 const ColorRGBA &color,
                                 const std::string &frame_id,
                                 const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = 1.0;
    vec_scale.z = 1.0;
    return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id, visualization_msgs::Marker::LINE_STRIP, lifetime);
}

Marker RosVizTools::newLineList(const double &scale,
                                const std::string &ns,
                                const int32_t &id,
                                const ColorRGBA &color,
                                const std::string &frame_id,
                                const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = 1.0;
    vec_scale.z = 1.0;
    return newMaker(vec_scale, defaultPose(), ns, id, color, frame_id, visualization_msgs::Marker::LINE_LIST, lifetime);
}

Marker RosVizTools::newCylinder(const geometry_msgs::Vector3 &scale,
                                const geometry_msgs::Pose &pose,
                                const std::string &ns,
                                const int32_t &id,
                                const ColorRGBA &color,
                                const std::string &frame_id,
                                const double &lifetime) {
    return newMaker(scale, pose, ns, id, color, frame_id, visualization_msgs::Marker::CYLINDER, lifetime);
}

Marker RosVizTools::newCube(const double &scale,
                            const geometry_msgs::Pose &pose,
                            const std::string &ns,
                            const int32_t &id,
                            const ColorRGBA &color,
                            const std::string &frame_id,
                            const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = scale;
    vec_scale.z = scale;
    return newMaker(vec_scale, pose, ns, id, color, frame_id, visualization_msgs::Marker::CUBE, lifetime);
}

Marker RosVizTools::newSphere(const double &scale,
                              const geometry_msgs::Pose &pose,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id,
                              const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = scale;
    vec_scale.y = scale;
    vec_scale.z = scale;
    return newMaker(vec_scale, pose, ns, id, color, frame_id, visualization_msgs::Marker::SPHERE, lifetime);
}

Marker RosVizTools::newArrow(const geometry_msgs::Vector3 &scale,
                             const geometry_msgs::Pose &pose,
                             const std::string &ns,
                             const int32_t &id,
                             const ColorRGBA &color,
                             const std::string &frame_id,
                             const double &lifetime) {
    return newMaker(scale, pose, ns, id, color, frame_id, visualization_msgs::Marker::ARROW, lifetime);
}

Marker RosVizTools::newText(const double &scale,
                            const geometry_msgs::Pose &pose,
                            const std::string &ns,
                            const int32_t &id,
                            const ColorRGBA &color,
                            const std::string &frame_id,
                            const double &lifetime) {
    geometry_msgs::Vector3 vec_scale;
    vec_scale.x = 1.0;
    vec_scale.y = 1.0;
    vec_scale.z = scale;
    return newMaker(vec_scale, pose, ns, id, color, frame_id, visualization_msgs::Marker::TEXT_VIEW_FACING, lifetime);
}

Marker RosVizTools::newFrame(const double &width,
                             const double &length,
                             const geometry_msgs::Pose &pose,
                             const std::string &ns,
                             const int32_t &id,
                             const std::string &frame_id,
                             const double &lifetime) {

    // line list marker
    Marker frame = newLineList(width, ns, id, ros_viz_tools::WHITE, frame_id, lifetime);

    // transform matrix - world origin to frame origin
    tf2::Transform trans_world_ori;
    tf2::convert(pose, trans_world_ori);

    // transform matrix - frame origin to key point of x, y, z axes
    tf2::Matrix3x3 mat(tf2::Quaternion(0, 0, 0, 1));
    tf2::Vector3 pos_x(length, 0, 0);
    tf2::Vector3 pos_y(0, length, 0);
    tf2::Vector3 pos_z(0, 0, length);
    tf2::Transform trans_ori_x(mat, pos_x);
    tf2::Transform trans_ori_y(mat, pos_y);
    tf2::Transform trans_ori_z(mat, pos_z);

    // transform matrix - world origin to key point of x, y, z axes
    tf2::Transform trans_world_x = trans_world_ori * trans_ori_x;
    tf2::Transform trans_world_y = trans_world_ori * trans_ori_y;
    tf2::Transform trans_world_z = trans_world_ori * trans_ori_z;

    geometry_msgs::Point p;

    // x axis
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    frame.points.push_back(p);
    frame.colors.push_back(RED);
    tf2::Vector3 r_wx = trans_world_x.getOrigin();
    p.x = r_wx[0];
    p.y = r_wx[1];
    p.z = r_wx[2];
    frame.points.push_back(p);
    frame.colors.push_back(RED);

    // y axis
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    frame.points.push_back(p);
    frame.colors.push_back(GREEN);
    tf2::Vector3 r_wy = trans_world_y.getOrigin();
    p.x = r_wy[0];
    p.y = r_wy[1];
    p.z = r_wy[2];
    frame.points.push_back(p);
    frame.colors.push_back(GREEN);

    // z axis
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    frame.points.push_back(p);
    frame.colors.push_back(BLUE);
    tf2::Vector3 r_wz = trans_world_z.getOrigin();
    p.x = r_wz[0];
    p.y = r_wz[1];
    p.z = r_wz[2];
    frame.points.push_back(p);
    frame.colors.push_back(BLUE);

    return frame;
}

geometry_msgs::Pose RosVizTools::defaultPose() {
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}
} // namespace ros_viz_tools