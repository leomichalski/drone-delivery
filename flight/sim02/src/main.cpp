#include <edra_msgs/ArucoDetection.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <inttypes.h>
#include <math.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

const int MODE_ARUCO_SEARCH = 1;
const int MODE_ARUCO_LAND = 2;  // precision landing
const int MODE_LAND = 3;
const int MODE_START_RETURN_HOME = 4;
const int MODE_RETURN_HOME = 5;
int mode_g = MODE_ARUCO_SEARCH;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g;

ros::Subscriber aruco_detection_sub;

ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;

// current_gps_location current_gps_location_msg;

/* This structure is a convenient way to format waypoints */
struct gnc_api_waypoint {
  float x;    ///< distance in x with respect to your reference frame
  float y;    ///< distance in y with respect to your reference frame
  float z;    ///< distance in z with respect to your reference frame
  float psi;  ///< rotation about the third axis of your reference frame
};

// get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state_g = *msg;
}

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu) {
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI / 180);
  geometry_msgs::Point current_pos_local;
  current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
  current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;

  // ROS_INFO("Local position %f %f %f",X, Y, Z);
}

// get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose_g = *msg;
  enu_2_local(current_pose_g);
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}

geometry_msgs::Point get_current_location() {
  geometry_msgs::Point current_pos_local;
  current_pos_local = enu_2_local(current_pose_g);
  return current_pos_local;
}

float get_current_heading() {
  return current_heading_g; 
}

// set orientation of the drone (drone should always be level)
// Heading input should match the ENU coordinate system
/* This function is used to specify the drone’s heading in the local reference
   frame. Psi is a counter clockwise rotation following the drone’s reference frame
   defined by the x axis through the right side of the drone with the y axis
   through the front of the drone. */
void set_heading(float heading) {
  local_desired_heading_g = heading;
  heading = heading + correction_heading_g + local_offset_g;

  ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = heading * (M_PI / 180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}

// set position to fly to in the local frame
/* This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. */
void set_destination(float x, float y, float z, float psi) {
  set_heading(psi);
  //transform map to local
  float deg2rad = (M_PI/180);
  float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
  float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
  float Zlocal = z;

  x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
  y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
  z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
  ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

  waypoint_g.pose.position.x = x;
  waypoint_g.pose.position.y = y;
  waypoint_g.pose.position.z = z;

  local_pos_pub.publish(waypoint_g);
  
}

/* Wait for connect is a function that will hold the program until communication with the FCU is established. */
int wait_for_connection() {
  ROS_INFO("Waiting for FCU connection");
  // wait for FCU connection
  while (ros::ok() && !current_state_g.connected) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (current_state_g.connected) {
    ROS_INFO("Connected to FCU");
    return 0;
  } else {
    ROS_INFO("Error connecting to drone");
    return -1;
  }
}

/* Wait for start will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station. */
int wait_for_start() {
  ROS_INFO("Waiting for user to set mode to GUIDED");
  while (ros::ok() && current_state_g.mode != "GUIDED") {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (current_state_g.mode == "GUIDED") {
    ROS_INFO("Mode set to GUIDED. Mission starting");
    return 0;
  } else {
    ROS_INFO("Error starting mission!!");
    return -1;
  }
}




/* This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to. */
int initialize_local_frame()
{
  //set the orientation of the local reference frame
  ROS_INFO("Initializing local coordinate system");
  local_offset_g = 0;
  for (int i = 1; i <= 30; i++) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    float q0 = current_pose_g.pose.pose.orientation.w;
    float q1 = current_pose_g.pose.pose.orientation.x;
    float q2 = current_pose_g.pose.pose.orientation.y;
    float q3 = current_pose_g.pose.pose.orientation.z;
    float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

    local_offset_g += psi*(180/M_PI);

    local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
    local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
    local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
    // ROS_INFO("current heading%d: %f", i, local_offset_g/i);
  }
  local_offset_pose_g.x = local_offset_pose_g.x/30;
  local_offset_pose_g.y = local_offset_pose_g.y/30;
  local_offset_pose_g.z = local_offset_pose_g.z/30;
  local_offset_g /= 30;
  ROS_INFO("Coordinate offset set");
  ROS_INFO("the X' axis is facing: %f", local_offset_g);
  return 0;
}

int arm() {
  //intitialize first waypoint of mission
  set_destination(0,0,0,0);
  for(int i=0; i<100; i++) {
    local_pos_pub.publish(waypoint_g);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  // arming
  ROS_INFO("Arming drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!current_state_g.armed && !arm_request.response.success && ros::ok()) {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    local_pos_pub.publish(waypoint_g);
  }
  if(arm_request.response.success) {
    ROS_INFO("Arming Successful");  
    return 0;
  } else {
    ROS_INFO("Arming failed with %d", arm_request.response.success);
    return -1;  
  }
}

/** The takeoff function will arm the drone and put the drone in a hover above the initial position. */
int takeoff(float takeoff_alt)
{
  //intitialize first waypoint of mission
  set_destination(0,0,takeoff_alt,0);
  for(int i=0; i<100; i++) {
    local_pos_pub.publish(waypoint_g);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  // arming
  ROS_INFO("Arming drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!current_state_g.armed && !arm_request.response.success && ros::ok()) {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    local_pos_pub.publish(waypoint_g);
  }
  if (arm_request.response.success) {
    ROS_INFO("Arming Successful");  
  } else {
    ROS_INFO("Arming failed with %d", arm_request.response.success);
    return -1;  
  }

  //request takeoff
  
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  if (takeoff_client.call(srv_takeoff)) {
    sleep(3);
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  } else {
    ROS_ERROR("Failed Takeoff");
    return -2;
  }
  sleep(2);
  return 0; 
}
/* This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. */
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01) {
  local_pos_pub.publish(waypoint_g);
  
  //check for correct position 
  float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
  float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
  float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
  float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
  // ROS_INFO("dMag %f", dMag);
  // ROS_INFO("current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
  // ROS_INFO("waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
  //check orientation
  float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
  float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
  
  float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

  // ROS_INFO("current heading %f", current_heading_g);
  // ROS_INFO("local_desired_heading_g %f", local_desired_heading_g);
  // ROS_INFO("current heading error %f", headingErr);

  if( dMag < pos_tolerance && headingErr < heading_tolerance) {
    return 1;
  } else {
    return 0;
  }
}
/** This function changes the mode of the drone to a user specified mode. This takes the mode as a string. ex. set_mode("GUIDED") */
int set_mode(std::string mode)
{
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0;
  srv_setMode.request.custom_mode = mode.c_str();
  if (set_mode_client.call(srv_setMode)) {
    ROS_INFO("setmode send ok");
  return 0;
  } else {
    ROS_ERROR("Failed SetMode");
    return -1;
  }
}

/* this function changes the mode of the drone to land */
int land() {
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success) {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  } else {
    ROS_ERROR("Landing failed");
    return -1;
  }
}

/* This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input */
int set_speed(float speed__mps) {
  mavros_msgs::CommandLong speed_cmd;
  speed_cmd.request.command = 178;
  speed_cmd.request.param1 = 1; // ground speed type 
  speed_cmd.request.param2 = speed__mps;
  speed_cmd.request.param3 = -1; // no throttle change
  speed_cmd.request.param4 = 0; // absolute speed
  ROS_INFO("setting speed to %f", speed__mps);
  if(command_client.call(speed_cmd)) {
    ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
    return 0;
  } else {
    ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
    ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
    return -1;
  }
  ROS_INFO("change speed result was %d ", speed_cmd.response.result);
  return 0;
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
  mode_g = MODE_ARUCO_LAND;  // todo: change this to arudo land mode (precision landing)
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle mission_node;

  std::string ros_namespace;
  if (!mission_node.hasParam("namespace")) {
    ROS_INFO("using default namespace");
  } else {
    mission_node.getParam("namespace", ros_namespace);
    ROS_INFO("using namespace %s", ros_namespace.c_str());
  }

  aruco_detection_sub = mission_node.subscribe((ros_namespace + "/aruco_detection").c_str(), 1, aruco_detection_cb);
  local_pos_pub = mission_node.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
  currentPos = mission_node.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
  state_sub = mission_node.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
  arming_client = mission_node.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
  land_client = mission_node.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
  set_mode_client = mission_node.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
  takeoff_client = mission_node.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
  command_client = mission_node.serviceClient<mavros_msgs::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());

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

  wait_for_connection();
  wait_for_start();
  initialize_local_frame();
  takeoff(3);

  ros::Rate rate(2.0);
  int i = 0;
  while (ros::ok()) {
    if (mode_g == MODE_ARUCO_SEARCH) {
      ros::spinOnce();
      rate.sleep();
      if (check_waypoint_reached(0.3)) {
        if (i < wpList.size()) {
          set_destination(wpList[i].x, wpList[i].y, wpList[i].z, wpList[i].psi);
          i++;
        } else {
          mode_g = MODE_START_RETURN_HOME;
        }
      }
    } else if (mode_g == MODE_ARUCO_LAND) {
      // TODO: implement precision landing
      land();
      ROS_INFO("Aruco landing started");
      break;
    } else if (mode_g == MODE_START_RETURN_HOME) {
      mode_g = MODE_RETURN_HOME;
      set_destination(wpList[0].x, wpList[0].y, wpList[0].z, wpList[0].psi);
      ROS_INFO("Returning to home");
    } else if (mode_g == MODE_RETURN_HOME) {
      ros::spinOnce();
      rate.sleep();
      if (check_waypoint_reached(0.3)) {
        land();
        ROS_INFO("Landing started");
        break;
      }
    }
  }

  return 0;
}

