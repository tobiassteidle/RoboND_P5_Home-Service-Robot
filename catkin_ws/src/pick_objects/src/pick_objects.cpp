#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction > MoveBaseClient;

bool move_to_position(MoveBaseClient & ac, double position_x, double orientation_w) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.orientation.w = orientation_w;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Successfully reached target position");
    return true;
  } else {
    ROS_INFO("Unable to reach target position.");
    return false;
  }
}

int main(int argc, char ** argv) {
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // Parameters from Parameter Server
  geometry_msgs::Point position_pickup = geometry_msgs::Point();
  n.getParam("/pickup_zone/x", position_pickup.x);
  n.getParam("/pickup_zone/y", position_pickup.y);
  ROS_INFO("Parameter /pickup_zone/x = %1.2f, /pickup_zone/y = %1.2f", (float)position_pickup.x, (float)position_pickup.y);

  geometry_msgs::Point position_dropoff = geometry_msgs::Point();
  n.getParam("/dropoff_zone/x", position_dropoff.x);
  n.getParam("/dropoff_zone/y", position_dropoff.y);
  ROS_INFO("Parameter /dropoff_zone/x = %1.2f, /dropoff_zone/y = %1.2f", (float)position_dropoff.x, (float)position_dropoff.y);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Move to first position
  bool done_position_1 = move_to_position(ac, position_pickup.x, position_pickup.y);

  // Wait 5 seconds in pickup zone
  ROS_INFO("Waiting in pickup zone.");
  ros::Duration(5.0).sleep();

  // Move to second position
  bool done_position_2 = move_to_position(ac, position_dropoff.x, position_dropoff.y);

  if (done_position_1 && done_position_2) {
    ROS_INFO("Successfully reached both positions.");
  }
  return 0;
}
