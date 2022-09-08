#include <inttypes.h>
#include <ros/ros.h>
#include <edra_msgs/ArucoDetection.h>


/*
#edra_msgs/ArucoDetection

int32 marker_id
float32[] marker_center
float32 elapsed_time
uint32 image_creation_time
*/
void aruco_detection_sub_callback(const edra_msgs::ArucoDetection::ConstPtr& msg) {
  int32_t marker_id = msg->marker_id;
  float center_x = msg->marker_center[0];
  float center_y = msg->marker_center[1];
  float elapsed_time = msg->elapsed_time;
  uint32_t image_creation_time = msg->image_creation_time;
  ROS_INFO("Marker ID: %d, Center: (%f, %f), Elapsed time: %f, Image creation time: %" PRIu32, marker_id, center_x, center_y, elapsed_time, image_creation_time);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detection_subscriber");
  ros::NodeHandle my_node;
  // topic, queue_size, callback
  ros::Subscriber sub_aruco_detection = my_node.subscribe("/aruco_detection", 1, aruco_detection_sub_callback);

  ros::Rate rate(2.0);
  int i = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    // ROS_DEBUG_NAMED("test_only", "Hello, world\n");
    ROS_INFO("Hello, world\n");
  }


  return 0;
}

