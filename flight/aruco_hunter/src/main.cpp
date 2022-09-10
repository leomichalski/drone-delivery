#include <edra_msgs/ArucoDetection.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <inttypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>
#include <sstream>
#include <string>

#include "GeographicLib/Geoid.hpp"

// state machine states
const int MODE_ARUCO_SEARCH = 2;
const int MODE_DELIVERY = 6;
const int MODE_RETURN_LAUNCH = 10;
const int _MODE_RETURN_LAUNCH = 11;
int mode_g = MODE_ARUCO_SEARCH;

// global variables related to the mission
const double TAKEOFF_HEIGHT = 15.0;  // in meters
const double PACKAGE_DELIVERY_HEIGHT = 2.0;  // in meters

// Publishers, subscribers, services
ros::Subscriber aruco_detection_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber state_sub;
ros::Publisher global_pos_pub;
ros::Subscriber global_pos_sub;

struct dest_struct {
  double latitude;
  double longitude;
  double altitude;
};

// other stuff
mavros_msgs::State current_state;
geographic_msgs::GeoPoseStamped global_position;
geographic_msgs::GeoPoseStamped next_dest_g;
bool global_position_received = false;

bool is_same_latitude(double lat1, double lat2) {
  // 0.00005 is a 5.5 meter(s) precision
  if (std::abs(lat1 - lat2) < 0.00005) {
    return true;
  }
  return false;
}
bool is_same_longitude(double lon1, double lon2) {
  // 0.00005 is a 5.5 meter(s) precision
  if (std::abs(lon1 - lon2) < 0.00005) {
    return true;
  }
  return false;
}
bool is_same_altitude(double alt1, double alt2) {
  // 1.0 is 1.0 meter(s) precision
  if (std::abs(alt1 - alt2) < 1.0) {
    return true;
  }
  return false;
}
bool is_same_coord(double lat1, double lon1, double alt1,
                   double lat2, double lon2, double alt2) {
  return is_same_latitude(lat1, lat2) && is_same_longitude(lon1, lon2) && is_same_altitude(alt1, alt2);
}

// TODO: make a lib to call GeographicLib
GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe

double calc_geoid_height(double lat, double lon) {
  return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib:: Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

void set_destination(double lat, double lon, double alt) {
  next_dest_g.header.stamp = ros::Time::now();
  next_dest_g.pose.position.latitude = lat;
  next_dest_g.pose.position.longitude = lon;
  next_dest_g.pose.position.altitude = alt;

  global_pos_pub.publish(next_dest_g);
}

void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  global_position.header.stamp = msg->header.stamp;
  global_position.pose.position.latitude = msg->latitude;
  global_position.pose.position.longitude = msg->longitude;
  global_position.pose.position.altitude = ellipsoid_height_to_amsl(
    msg->latitude,
    msg->longitude,
    msg->altitude
  );
  global_position_received = true;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

void aruco_detection_cb(const edra_msgs::ArucoDetection::ConstPtr& msg) {
  int32_t marker_id = msg->marker_id;
  float center_x = msg->marker_center[0];
  float center_y = msg->marker_center[1];
  float elapsed_time = msg->elapsed_time;
  uint32_t image_creation_time = msg->image_creation_time;
  // TODO: if the mav is within a 100m radius of the current gps location, then we have found the target
  if (mode_g == MODE_ARUCO_SEARCH) {
    mode_g = MODE_DELIVERY;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  aruco_detection_sub = nh.subscribe<edra_msgs::ArucoDetection>("/aruco_detection", 1, aruco_detection_cb);

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
  global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_position_cb);

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
  ROS_INFO(
    "Got global position: lat=%f, long=%f, alt=%f",
    global_position.pose.position.latitude,
    global_position.pose.position.longitude,
    global_position.pose.position.altitude
  );

  // INITIAL POSITION AND GOALS

  std::vector<dest_struct> dest_list;
  dest_struct dest;
  // don't takeoff, just stay in your position
  const uint64_t DEST_LIST_IDX_INITIAL_POSITION = dest_list.size();
  dest.latitude = global_position.pose.position.latitude;
  dest.longitude = global_position.pose.position.longitude;
  dest.altitude = global_position.pose.position.altitude;
  // dest.header_stamp = ros::Time::now();
  dest_list.push_back(dest);
  // takeoff
  // TODO: calculate if the drone is already in the air before taking off (get
  // the current place (netherlands, FGA) altitude from lat and long, then check
  // if it's higher than the current MAV altitude)
  const uint64_t DEST_LIST_IDX_TAKEOFF = dest_list.size();
  dest.latitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude;
  dest.longitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude;
  dest.altitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude + TAKEOFF_HEIGHT;
  dest_list.push_back(dest);

  // read mission waypoints from a file
  std::ifstream source;
  source.open("waypoints.txt", std::ios_base::in);
  for (std::string line; std::getline(source, line);) {
    std::istringstream in(line);  // make a stream for the line itself
    in >> dest.latitude >> dest.longitude;
    dest_list.push_back(dest);
  }

  // return to launch
  const uint64_t DEST_LIST_IDX_RETURNING_TO_LAUNCH = dest_list.size();
  dest.latitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude;
  dest.longitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude;
  dest.altitude = dest_list[DEST_LIST_IDX_TAKEOFF].altitude;
  dest_list.push_back(dest);
  // land
  const uint64_t DEST_LIST_IDX_LAND = dest_list.size();
  dest.latitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude;
  dest.longitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude;
  dest.altitude = dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude;
  dest_list.push_back(dest);

  // send a few setpoints before starting
  for (int i = 0; ros::ok() && i < 20; ++i) {
    set_destination(
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude
    );
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
        ROS_INFO(
            "Failed to send 'set mode to OFFBOARD' message, trying again in 5 "
            "seconds");
      }
      last_request = ros::Time::now();
    }
    set_destination(
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude
    );

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
    set_destination(
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].latitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].longitude,
      dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude
    );

    ros::spinOnce();
    rate.sleep();
  }

  // TAKEOFF
  int dest_list_idx = DEST_LIST_IDX_TAKEOFF;
  while (ros::ok()) {
    set_destination(
      dest_list[dest_list_idx].latitude,
      dest_list[dest_list_idx].longitude,
      dest_list[dest_list_idx].altitude
    );

    ros::spinOnce();
    rate.sleep();
    if (is_same_altitude(global_position.pose.position.altitude,
                         dest_list[dest_list_idx].altitude)) {
      ROS_INFO("UAV at takeoff altitude");
      dest_list_idx++;
      break;
    }
  }

  ros::Time start_delivery_time;
  while (ros::ok()) {
    if (mode_g == MODE_ARUCO_SEARCH || mode_g == _MODE_RETURN_LAUNCH) {
      set_destination(
        dest_list[dest_list_idx].latitude,
        dest_list[dest_list_idx].longitude,
        dest_list[dest_list_idx].altitude
      );

      if (is_same_coord(global_position.pose.position.latitude,
                        global_position.pose.position.longitude,
                        global_position.pose.position.altitude,
                        next_dest_g.pose.position.latitude,
                        next_dest_g.pose.position.longitude,
                        next_dest_g.pose.position.altitude)) {
        dest_list_idx++;
        if (dest_list_idx >= dest_list.size()) {
          break;
        }
        ROS_INFO(
          "UAV heading to lat=%f, lon=%f, alt=%f",
          dest_list[dest_list_idx].latitude,
          dest_list[dest_list_idx].longitude,
          dest_list[dest_list_idx].altitude
        );
      }
      ros::spinOnce();
      rate.sleep();
    } else if (mode_g == MODE_DELIVERY) {
      set_destination(
        global_position.pose.position.latitude,
        global_position.pose.position.longitude,
        dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude + PACKAGE_DELIVERY_HEIGHT
      );
      ROS_INFO("Getting down to deliver the package");

      ros::spinOnce();
      rate.sleep();
      if (is_same_coord(global_position.pose.position.latitude,
                        global_position.pose.position.longitude,
                        global_position.pose.position.altitude,
                        next_dest_g.pose.position.latitude,
                        next_dest_g.pose.position.longitude,
                        next_dest_g.pose.position.altitude)) {
        ROS_INFO("UAV at delivery altitude");
        // keep the UAV at the same altitude for a while
        // 15 seconds
        start_delivery_time = ros::Time::now();
        while ((ros::ok()) && (ros::Time::now() - start_delivery_time < ros::Duration(15))) {
          set_destination(
            global_position.pose.position.latitude,
            global_position.pose.position.longitude,
            dest_list[DEST_LIST_IDX_INITIAL_POSITION].altitude + PACKAGE_DELIVERY_HEIGHT
          );
          ros::spinOnce();
          rate.sleep();
        }
        mode_g = MODE_RETURN_LAUNCH;
      }
    } else if (mode_g == MODE_RETURN_LAUNCH) {
      ROS_INFO("Returning to launch position");
      mode_g = _MODE_RETURN_LAUNCH;
      dest_list_idx = DEST_LIST_IDX_RETURNING_TO_LAUNCH;
    }
  }

  return 0;
}
