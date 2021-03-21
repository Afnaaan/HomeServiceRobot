#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

double pickUp[2] = {-1.0, 3.0};
double dropOff[2] = {4.0, 1.0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pickup_zone;
    move_base_msgs::MoveBaseGoal drop_off_zone;

    // set up the frame parameters
    pickup_zone.target_pose.header.frame_id = "map";
    pickup_zone.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup_zone.target_pose.pose.position.x = pickUp[0];
    pickup_zone.target_pose.pose.position.y = pickUp[1];
    pickup_zone.target_pose.pose.orientation.w = 1.0;

    // Display a message that we In our way to pickup destination
    ROS_INFO("In our way to pickup zone");
    ac.sendGoal(pickup_zone);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its pickup_zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Successfully reached our pickup zone");
    else
        ROS_INFO("Failed to reach our pickup zone for some reason");

    ros::Duration(5.0).sleep(); 
  	// Wait 5 sec after reaching the pickup zone
    ac.waitForServer(ros::Duration(5.0));

    // set up the frame parameters
    drop_off_zone.target_pose.header.frame_id = "map";
    drop_off_zone.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    drop_off_zone.target_pose.pose.position.x = dropOff[0];
    drop_off_zone.target_pose.pose.position.y = dropOff[1];
    drop_off_zone.target_pose.pose.orientation.w = 1.0;

    // Display a message that we In our way to drop off destination
    ROS_INFO("In our way to drop off zone");
    ac.sendGoal(drop_off_zone);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its drop_off_zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Successfully reached our drop off zone");
    else
        ROS_INFO("Failed to reach our drop off zone for some reason");

    return 0;
}