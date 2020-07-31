#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>

void visualize_marker(ros::Publisher & marker_pub, visualization_msgs::Marker & marker, int32_t action, geometry_msgs::Point position) {
  // Set the marker action.
  marker.action = action;

  // Set the pose of the marker.
  marker.pose.position = position;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker_pub.publish(marker);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1);

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

    ROS_INFO("Publish marker on pickup zone.");
    geometry_msgs::Point position_pickup = geometry_msgs::Point();

    // Publish marker to pickup zone
    visualize_marker(marker_pub, marker, visualization_msgs::Marker::ADD, position_pickup);

    // Wait for 5 Seconds
    ROS_INFO("Wait for 5 seconds...");
    ros::Duration(5.0).sleep();

    // Hide Marker
    ROS_INFO("Hide Marker.");
    visualize_marker(marker_pub, marker, visualization_msgs::Marker::DELETE, position_pickup);

    // Wait for 5 Seconds
    ROS_INFO("Wait for 5 seconds...");
    ros::Duration(5.0).sleep();

    // Publish marker to drop off zone
    ROS_INFO("Publish marker at the drop off zone.");
    geometry_msgs::Point position_dropoff = geometry_msgs::Point();
    position_dropoff.x = 1.0f;
    position_dropoff.y = 1.0f;
    marker.lifetime = ros::Duration();

    visualize_marker(marker_pub, marker, visualization_msgs::Marker::ADD, position_dropoff);

    while (ros::ok()) {
      r.sleep();
    }
  }
}
