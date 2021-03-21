#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

double pickUp[2] = {-1.0, 3.0};
double dropOff[2] = {4.0, 1.0};

bool is_drop_off_reached = false;
bool is_deleting_marker = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Get pose information
  ROS_INFO("odomCallback");
  double current_pose_x = msg->pose.pose.position.x;
  double current_pose_y = msg->pose.pose.position.y;
  if ((std::abs(pickUp[0] - current_pose_x) + std::abs(pickUp[1] - current_pose_y)) < 0.3)
  {
    is_deleting_marker = true;
    ROS_INFO("delete flag");
  }
  else if ((std::abs(dropOff[0] - current_pose_x) + std::abs(dropOff[1] - current_pose_y)) < 0.3)
  {
    is_drop_off_reached = true;
    ROS_INFO("is_drop_off_reached flag");
  }
  // ROS_INFO("Position-> x: [%f], y: [%f]", current_pose_x, current_pose_x);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    if (!is_drop_off_reached && !is_deleting_marker)
    {
      ROS_INFO("Adding pickup marker");
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
      is_deleting_marker = false;
      marker.pose.position.x = pickUp[0];
      marker.pose.position.y = pickUp[1];
      marker_pub.publish(marker);
    }

    if(is_deleting_marker)
    {
      ROS_INFO("Deleting pickup marker");
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      sleep(5);
    }

    if(is_drop_off_reached)
    {
      ROS_INFO("Adding drop off marker");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOff[0];
      marker.pose.position.y = dropOff[1];
      marker.pose.position.z = 0;
      marker_pub.publish(marker);
      sleep(5);
      is_drop_off_reached = false;
    }
    ros::spinOnce();
  }
}