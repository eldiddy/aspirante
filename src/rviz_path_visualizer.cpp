#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
void odom_callback(const nav_msgs::Odometry::ConstPtr&);
visualization_msgs::Marker initialize_marker();

int main (int argc, char ** argv) {
    ros::init(argc, argv, "rviz_path_visualizer");

    ros::NodeHandle node;
    // marker publisher for rviz
    marker_pub = node.advertise<visualization_msgs::Marker>("/visualization_marker", 100);

    ros::Subscriber pose_sub = node.subscribe<nav_msgs::Odometry>("/odom", 100, odom_callback);
    ros::spin();
    return 0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    static visualization_msgs::Marker line_strip = initialize_marker();

    line_strip.points.push_back(msg->pose.pose.position);
    line_strip.header.stamp = ros::Time::now();
    marker_pub.publish(line_strip);
}

visualization_msgs::Marker initialize_marker() {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.ns = "robot_path";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.4; // thickness of the marker

    // color of the marker
    line_strip.color.a = 0.5;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;

    return line_strip;
}