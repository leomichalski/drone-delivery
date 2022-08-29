#include <ros/console.h>
#include <gnc_functions.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;

  init_publisher_subscriber(gnc_node);

  ROS_DEBUG_NAMED("test_only", "waiting 4 connection");
  wait_for_connection();
  ROS_DEBUG_NAMED("test_only", "connected");

  wait_for_start();
  ROS_DEBUG_NAMED("test_only", "started");

  initialize_local_frame();
  ROS_DEBUG_NAMED("test_only", "initialized");

  takeoff(3);

  std::vector<gnc_api_waypoint> wpList;
  gnc_api_waypoint nextWp;
  nextWp.x = 0;
  nextWp.y = 0;
  nextWp.z = 3;
  nextWp.psi = 0;
  wpList.push_back(nextWp);
  nextWp.x = 5;
  nextWp.y = 0;
  nextWp.z = 3;
  nextWp.psi = -90;
  wpList.push_back(nextWp);
  nextWp.x = 5;
  nextWp.y = 5;
  nextWp.z = 3;
  nextWp.psi = 0;
  wpList.push_back(nextWp);
  nextWp.x = 0;
  nextWp.y = 5;
  nextWp.z = 3;
  nextWp.psi = 90;
  wpList.push_back(nextWp);
  nextWp.x = 0;
  nextWp.y = 0;
  nextWp.z = 3;
  nextWp.psi = 180;
  wpList.push_back(nextWp);
  nextWp.x = 0;
  nextWp.y = 0;
  nextWp.z = 3;
  nextWp.psi = 0;
  wpList.push_back(nextWp);

  ros::Rate rate(2.0);
  int i = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (check_waypoint_reached(0.3)) {
      if (i < wpList.size()) {
        set_destination(wpList[i].x, wpList[i].y, wpList[i].z, wpList[i].psi);
        i++;
      } else {
        land();
      }
    }
  }
  return 0;
}