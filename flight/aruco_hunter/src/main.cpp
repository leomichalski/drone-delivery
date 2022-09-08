#include <edra_msgs/ArucoDetection.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <inttypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "GeographicLib/Geoid.hpp"

// state machine states
const int MODE_ARUCO_SEARCH = 2;
const int MODE_START_DELIVERY = 4;
const int MODE_DELIVERY = 6;
const int MODE_LAND = 8;
const int MODE_START_RETURN_HOME = 10;
const int MODE_RETURN_HOME = 12;
int mode_g = MODE_ARUCO_SEARCH;

// global variables related to the waypoints
const double TAKEOFF_ALTITUDE = 5.0;  // in meters

// other stuff
mavros_msgs::State current_state;
geographic_msgs::GeoPoseStamped global_position;
bool global_position_received = false;
// TODO: make a lib to call GeographicLib
GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe

double calc_geoid_height(double lat, double lon) {
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position.header.stamp = msg->header.stamp;
    global_position.pose.position.latitude = msg->latitude;
    global_position.pose.position.longitude = msg->longitude;
    global_position.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    global_position_received = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

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

int main(int argc, char **argv) {


  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber aruco_detection_sub = nh.subscribe<edra_msgs::ArucoDetection>("/aruco_detection", 1, aruco_detection_cb);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
  ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_position_cb);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);
  // ros::Rate rate(20.0);
  ros::Time last_request;
  uint16_t retry_counter;

  // WAIT FOR FCU CONNECTION
  ROS_INFO("Waiting for FCU connection...");
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected");

  // WAIT FOR GPS INFORMATION
  ROS_INFO("Waiting for GPS signal...");
  while (ros::ok() && !global_position_received) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Got global position: lat=%f, long=%f, alt=%f", global_position.pose.position.latitude, global_position.pose.position.longitude, global_position.pose.position.altitude);

  // INITIAL POSITION AND GOALS

  std::vector<geographic_msgs::GeoPoseStamped> pose_list;
  geographic_msgs::GeoPoseStamped pose;
  // don't takeoff, just stay in your position
  const uint64_t POSE_LIST_IDX_INITIAL_POSITION = pose_list.size();
  pose.pose.position.latitude = global_position.pose.position.latitude;
  pose.pose.position.longitude = global_position.pose.position.longitude;
  pose.pose.position.altitude = global_position.pose.position.altitude;
  pose_list.push_back(pose);
  // takeoff
  // TODO: calculate if the drone is already in the air before taking off (get the current place (netherlands, FGA) altitude from lat and long, then check if it's higher than the current MAV altitude)
  const uint64_t POSE_LIST_IDX_TAKEOFF = pose_list.size();
  pose.pose.position.latitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.latitude;
  pose.pose.position.longitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.longitude;
  pose.pose.position.altitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.altitude + TAKEOFF_ALTITUDE;
  pose_list.push_back(pose);
  // // just another goal
  // pose.pose.position.latitude = ;
  // pose.pose.position.longitude = ;
  // pose_list.push_back(pose);
  // // just another goal
  // pose.pose.position.latitude = ;
  // pose.pose.position.longitude = ;
  // pose_list.push_back(pose);
  // return to launch
  const uint64_t POSE_LIST_IDX_RETURNING_TO_LAUNCH = pose_list.size();
  pose.pose.position.latitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.latitude;
  pose.pose.position.longitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.longitude;
  pose.pose.position.altitude = pose_list[POSE_LIST_IDX_TAKEOFF].pose.position.altitude;
  pose_list.push_back(pose);
  // land
  const uint64_t POSE_LIST_IDX_LAND = pose_list.size();
  pose.pose.position.latitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.latitude;
  pose.pose.position.longitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.longitude;
  pose.pose.position.altitude = pose_list[POSE_LIST_IDX_INITIAL_POSITION].pose.position.altitude;
  pose_list.push_back(pose);

  // // WP 1
  // wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  // wp.command = mavros_msgs::CommandCode::NAV_LOITER_TIME;
  // // WP 3
  // wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
  // wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;

  // send a few setpoints before starting
  for (int i = 0; ros::ok() && i < 20; ++i) {
    pose.header.stamp = ros::Time::now();
    global_pos_pub.publish(pose_list[POSE_LIST_IDX_INITIAL_POSITION]);
    ros::spinOnce();
    rate.sleep();
  }

  // SET MODE
  // set_mode()
  ROS_INFO("Waiting offboard mode");

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.base_mode = 0;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  retry_counter = 0;
  while (ros::ok()) {
    if (current_state.mode == "OFFBOARD") {
      ROS_INFO("Vehicle mode set to OFFBOARD");
      break;
    }
    ROS_INFO("Waiting 'set mode to OFFBOARD'...");
    // if takes more than 5 seconds to arm,
    if ((retry_counter == 0) || (ros::Time::now() - last_request > ros::Duration(5))) {
      retry_counter++;
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Sent 'set mode to OFFBOARD' message");
      } else {
        ROS_INFO("Failed to send 'set mode to OFFBOARD' message, trying again in 5 seconds");
      }
      last_request = ros::Time::now();
    }
    global_pos_pub.publish(pose_list[POSE_LIST_IDX_INITIAL_POSITION]);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("OFFBOARD enabled");



  // ARM VEHICLE
  // arm();
  ROS_INFO("Arming vehicle");

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  retry_counter = 0;
  while (ros::ok()) {
    if (current_state.armed) {
      ROS_INFO("Vehicle armed");
      break;
    }
    ROS_INFO("Waiting arm...");
    // if takes more than 5 seconds to arm,
    if ((retry_counter == 0) || (ros::Time::now() - last_request > ros::Duration(5))) {
      retry_counter++;
      if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Sent arming message");
      } else {
        ROS_INFO("Failed to sent arming message, trying again in 5 seconds");
      }
      last_request = ros::Time::now();
    }
    global_pos_pub.publish(pose_list[POSE_LIST_IDX_INITIAL_POSITION]);
    ros::spinOnce();
    rate.sleep();
  }

  // TODO: better takeoff logic
  // uint64_t pose_list_idx = 0;
  ROS_INFO("Taking off to: lat=%f, long=%f, alt=%f",
           pose_list[POSE_LIST_IDX_TAKEOFF].pose.position.latitude,
           pose_list[POSE_LIST_IDX_TAKEOFF].pose.position.longitude,
           pose_list[POSE_LIST_IDX_TAKEOFF].pose.position.altitude);
  while (ros::ok()) {
    pose_list[POSE_LIST_IDX_TAKEOFF].header.stamp = ros::Time::now();
    global_pos_pub.publish(pose_list[POSE_LIST_IDX_TAKEOFF]);
    ros::spinOnce();
    ROS_INFO_THROTTLE(1, "UAV at: lat=%f, long=%f, alt=%f",
                      global_position.pose.position.latitude,
                      global_position.pose.position.longitude,
                      global_position.pose.position.altitude);
    }

  // int i = 0;
  // while (ros::ok()) {
  //   i++;
  //   ros::spinOnce();
  //   rate.sleep();
  // }

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
