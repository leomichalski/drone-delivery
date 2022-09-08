/*
roslaunch px4 mavros_posix_sitl.launch

// simulation only params
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
*/
#include <edra_msgs/ArucoDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <inttypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <ros/ros.h>

#include <list>

// TODO: migrar do ardupilot para o px4
// TODO: usar setposition GPS em vez desta simplificação (local)
// TODO: usar "/mavros/setpoint_position/global" e "/mavros/global_position/global" em vez do "/mavros/setpoint_position/local" e "/mavros/global_position/local"

const int MODE_ARUCO_SEARCH = 2;
const int MODE_START_DELIVERY = 4;
const int MODE_DELIVERY = 6;
const int MODE_LAND = 8;
const int MODE_START_RETURN_HOME = 10;
const int MODE_RETURN_HOME = 12;
int mode_g = MODE_ARUCO_SEARCH;

void aruco_detection_cb(const edra_msgs::ArucoDetection::ConstPtr& msg) {
  int32_t marker_id = msg->marker_id;
  float center_x = msg->marker_center[0];
  float center_y = msg->marker_center[1];
  float elapsed_time = msg->elapsed_time;
  uint32_t image_creation_time = msg->image_creation_time;
  // ROS_INFO("Marker ID: %d, Center: (%f, %f), Elapsed time: %f, Image creation time: %" PRIu32, marker_id, center_x, center_y, elapsed_time, image_creation_time);
  // TODO: if the mav is within a 100m radius of the current gps location, then we have found the target
  // float deltaX = abs(center_x - current_pose_g.pose.pose.position.x);
  if (mode_g == MODE_ARUCO_SEARCH) {
    mode_g = MODE_START_DELIVERY;
  }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv) {


  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber aruco_detection_sub = nh.subscribe<edra_msgs::ArucoDetection>("/aruco_detection", 1, aruco_detection_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
  mavros_msgs::WaypointPush wp_list;
  mavros_msgs::Waypoint wp;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  ros::Time last_request;

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // WP 0
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp.is_current = true;
  wp.autocontinue = true;
  wp.x_lat = 52.171974;
  wp.y_long = 4.417091;
  wp.z_alt = 10;
  wp_list.request.waypoints.push_back(wp);
  // WP 1
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_LOITER_TIME;
  wp.is_current = false;
  wp.autocontinue = true;
  wp.x_lat = 52.1704061;
  wp.y_long = 4.4198957;
  wp.z_alt = 20;
  wp.param1 = 10;  // time to loiter at waypoint (seconds - decimal
  wp.param3 = 2;  // Radius around waypoint, in meters. Specify as a positive value to loiter clockwise, negative to move counter-clockwise.
  wp.param4 = 1;  // Desired yaw angle.
  wp_list.request.waypoints.push_back(wp);

  // WP 2
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current = false;
  wp.autocontinue = true;
  wp.x_lat = 52.1719317;
  wp.y_long = 4.4210099;
  wp.z_alt = 20;
  wp_list.request.waypoints.push_back(wp);

  // WP 3
  wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
  wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
  wp.is_current = false;
  wp.autocontinue = true;
  wp.x_lat = 0;
  wp.y_long = 0;
  wp.z_alt = 0;
  wp_list.request.waypoints.push_back(wp);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // wait for FCU connection
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // SET MODE
  // ROS_INFO("Waiting offboard mode");
  // last_request = ros::Time::now();

  // mavros_msgs::SetMode offb_set_mode;
  // offb_set_mode.request.custom_mode = "OFFBOARD";

  // while (ros::ok() && current_state.mode != "OFFBOARD") {
  //   if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
  //     continue;
  //   }
  //   if (set_mode_client.call(offb_set_mode) &&
  //       offb_set_mode.response.mode_sent) {
  //     ROS_INFO("Offboard should be enabled...");
  //   }
  //   last_request = ros::Time::now();

  //   local_pos_pub.publish(pose);

  //   ros::spinOnce();
  //   rate.sleep();
  // }
  // ROS_INFO("Offboard enabled");

  // ARM VEHICLE
  ROS_INFO("Arming vehicle");

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  while (ros::ok() && !current_state.armed) {
    if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
      continue;
    }
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("Vehicle should be armed...");
    }
    last_request = ros::Time::now();

    // local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle armed");



  // SEND WAYPOINTS
  ROS_INFO("Waiting for waypoint service");
  last_request = ros::Time::now();
  while (ros::ok()) {
    if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
      continue;
    }
    if (wp_client.call(wp_list)) {
      ROS_INFO("Waypoints pushed");
      break;
    }
    ROS_INFO("Waiting for waypoint service...");
    last_request = ros::Time::now();
    // local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // SET MODE
  ROS_INFO("Waiting auto.mission mode");
  last_request = ros::Time::now();

  mavros_msgs::SetMode auto_set_mode;
  auto_set_mode.request.custom_mode = "AUTO.MISSION";

  while (ros::ok() && current_state.mode != "AUTO.MISSION") {
    if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
      continue;
    }

    // if (set_mode_client.call(auto_set_mode) &&
    // auto_set_mode.response.success) {
    if (set_mode_client.call(auto_set_mode) && auto_set_mode.response.mode_sent) {
      ROS_INFO("Attempting to set AUTO.MISSION mode...");
    }
    last_request = ros::Time::now();
    // ROS_INFO("Waiting for waypoint service");
    // local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("AUTO.MISSION enabled");

  int i = 0;
  while (ros::ok()) {
    i++;
    ros::spinOnce();
    rate.sleep();
  }

//   ros::Rate rate(2.0);
//   int i = 0;
//   while (ros::ok()) {
//     if (mode_g == MODE_ARUCO_SEARCH) {
//       ros::spinOnce();
//       rate.sleep();
//       if (check_waypoint_reached(0.3)) {
//         if (i < wpList.size()) {
//           set_destination(wpList[i].x, wpList[i].y, wpList[i].z, wpList[i].psi);
//           i++;
//         } else {
//           mode_g = MODE_START_RETURN_HOME;
//         }
//       }
//     } else if (mode_g == MODE_START_DELIVERY) {
//       // TODO: implement precision descending until 2 meters high (not landing)
//       // TODO: jeito certo de pegar a position atual do drone
//       current_position = get_current_location();
//       set_destination(current_position.x, current_position.y, 2, wpList[i].psi);
//       // land();
//       ROS_INFO("Getting down to deliver the package");
//       // break;
//       mode_g = MODE_DELIVERY;
//     } else if (mode_g == MODE_DELIVERY) {
//       ros::spinOnce();
//       rate.sleep();
//       if (check_waypoint_reached(0.3)) {
//         ROS_INFO("Delivery started");
//         // HOVER FOR SOME SECONDS, THEN RETURN HOME
//         ros::Duration(15).sleep();
//         mode_g = MODE_START_RETURN_HOME;
//       }

//     } else if (mode_g == MODE_START_RETURN_HOME) {
//       mode_g = MODE_RETURN_HOME;
//       set_destination(wpList[0].x, wpList[0].y, wpList[0].z, wpList[0].psi);
//       ROS_INFO("Returning to home");
//     } else if (mode_g == MODE_RETURN_HOME) {
//       ros::spinOnce();
//       rate.sleep();
//       if (check_waypoint_reached(0.3)) {
//         land();
//         ROS_INFO("Landing started");
//         break;
//       }
//     }
//   }

  return 0;
}

