#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pubWaypoint;

void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg)
{
  for (auto& pose : pathMsg->poses)
  {
    geometry_msgs::PointStamped waypointMsg;
    waypointMsg.header.stamp = ros::Time::now();
    waypointMsg.header.frame_id = "/map"; // Assuming the path is in the map frame
    waypointMsg.point.x = pose.pose.position.x;
    waypointMsg.point.y = pose.pose.position.y;
    waypointMsg.point.z = 0; // Z coordinate is 0

    pubWaypoint.publish(waypointMsg);
    ros::Duration(0.1).sleep(); // Wait for a short time before publishing the next waypoint
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "globalPathConverter");
  ros::NodeHandle nh;

  pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/global_path", 1, pathCallback);

  ros::spin();

  return 0;
}

