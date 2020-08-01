#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

enum State { Initialize, Idle, Pickup, Drop };
State state = Initialize;

void visualize_marker(ros::Publisher &marker_pub, visualization_msgs::Marker &marker, int32_t action, geometry_msgs::Point position) {
  // Set the marker action.
  marker.action = action;

  // Set the pose of the marker.
  marker.pose.position = position;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker_pub.publish(marker);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const float pos_x = (float)odom->pose.pose.position.x;
  const float pos_y = (float)odom->pose.pose.position.y;
  //ROS_INFO("Current Position x = %1.2f, y = %1.2f", pos_x, pos_y);
}

void action_callback(const std_msgs::Int8::ConstPtr &action)
{
    ROS_INFO("Do Action: %d", action->data);
    if(action->data == 1)
    {
        state = Pickup;
    }
    else if(action->data == 2)
    {
        state = Drop;
    }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_callback);
  ros::Subscriber action_sub = n.subscribe("/my_robot/action_publisher", 10, action_callback);
  ros::Publisher marker_pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1);

  // Parameters from Parameter Server
  float position_tolerance;
  n.getParam("position_tolerance", position_tolerance);
  ROS_INFO("Parameter position_tolerance = %1.2f", (float)position_tolerance);

  geometry_msgs::Point position_pickup = geometry_msgs::Point();
  n.getParam("/pickup_zone/x", position_pickup.x);
  n.getParam("/pickup_zone/y", position_pickup.y);
  ROS_INFO("Parameter /pickup_zone/x = %1.2f, /pickup_zone/y = %1.2f", (float)position_pickup.x, (float)position_pickup.y);

  geometry_msgs::Point position_dropoff = geometry_msgs::Point();
  n.getParam("/dropoff_zone/x", position_dropoff.x);
  n.getParam("/dropoff_zone/y", position_dropoff.y);
  ROS_INFO("Parameter /dropoff_zone/x = %1.2f, /dropoff_zone/y = %1.2f", (float)position_dropoff.x, (float)position_dropoff.y);

  if (ros::ok()) {
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE,
    marker.type = visualization_msgs::Marker::CUBE;

    // Do never auto-delete the markers
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }






/*
    // Wait for 5 Seconds
    ROS_INFO("Wait for 5 seconds...");
    ros::Duration(5.0).sleep();

    // Hide Marker


    // Wait for 5 Seconds
    ROS_INFO("Wait for 5 seconds...");
    ros::Duration(5.0).sleep();

    // Publish marker to drop off zone
    ROS_INFO("Publish marker at the drop off zone.");
    marker.lifetime = ros::Duration();

    visualize_marker(marker_pub, marker, visualization_msgs::Marker::ADD, position_dropoff);

*/

    while (ros::ok())
    {
        switch(state)
        {
            case Idle:
                // Do Nothing
                break;

            case Initialize:
                ROS_INFO("Publish marker on pickup zone.");
                visualize_marker(marker_pub, marker, visualization_msgs::Marker::ADD, position_pickup);
                state = Idle;
                break;

            case Pickup:
                ROS_INFO("Pickup Object.");
                visualize_marker(marker_pub, marker, visualization_msgs::Marker::DELETE, position_pickup);
                state = Idle;
                break;

            case Drop:
                ROS_INFO("Publish marker at the drop off zone.");
                visualize_marker(marker_pub, marker, visualization_msgs::Marker::ADD, position_dropoff);
                state = Idle;
                break;
        }

      ros::spinOnce();
    }



  }
}
