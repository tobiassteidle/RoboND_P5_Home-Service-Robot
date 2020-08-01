#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction > MoveBaseClient;

#define DO_PICKUP 1
#define DO_DROPOFF 2

struct MovementTarget {   // Declare PERSON struct type
    int target_id;
    geometry_msgs::Point target_point;
};

bool move_to_position(MoveBaseClient &ac, MovementTarget &movement_target) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = movement_target.target_point.x;
  goal.target_pose.pose.orientation.w = movement_target.target_point.y;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal, do path planning...");
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

  // Create Publisher for pickup and dropoff
  ros::Publisher action_publisher = n.advertise<std_msgs::Int8>("/my_robot/action_publisher", 10);

  // Parameters from Parameter Server
  MovementTarget pickup_target;
  n.getParam("/pickup_zone/id", pickup_target.target_id);
  n.getParam("/pickup_zone/x", pickup_target.target_point.x);
  n.getParam("/pickup_zone/y", pickup_target.target_point.y);
  ROS_INFO("Parameter /pickup_zone/x = %1.2f, /pickup_zone/y = %1.2f", (float)pickup_target.target_point.x, (float)pickup_target.target_point.y);

  MovementTarget dropoff_target;
  n.getParam("/dropoff_zone/id", dropoff_target.target_id);
  n.getParam("/dropoff_zone/x", dropoff_target.target_point.x);
  n.getParam("/dropoff_zone/y", dropoff_target.target_point.y);
  ROS_INFO("Parameter /dropoff_zone/x = %1.2f, /dropoff_zone/y = %1.2f", (float)dropoff_target.target_point.x, (float)dropoff_target.target_point.y);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Move to first position
  bool done_position_1 = move_to_position(ac, pickup_target);
  if(done_position_1)
  {
    std_msgs::Int8 msg;
    msg.data = DO_PICKUP;
    action_publisher.publish(msg);
  }

  // Wait 5 seconds in pickup zone
  ROS_INFO("Waiting in pickup zone.");
  ros::Duration(5.0).sleep();

  // Move to second position
  bool done_position_2 = move_to_position(ac, dropoff_target);

  if (done_position_1 && done_position_2) {
    ROS_INFO("Successfully reached both positions.");
    std_msgs::Int8 msg;
    msg.data = DO_DROPOFF;
    action_publisher.publish(msg);
  }
  return 0;
}
