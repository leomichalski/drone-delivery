/*
roslaunch px4 mavros_posix_sitl.launch

// simulation only params
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
*/
#include <edra_msgs/ArucoDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <inttypes.h>

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


/*
  std::vector<gnc_api_waypoint> wpList;
  // TODO: ler waypoints automaticamente a partir do xml
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

*/
int main(int argc, char **argv) {


  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber aruco_detection_sub = nh.subscribe("/aruco_detection", 1, aruco_detection_cb);
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

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

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ROS_INFO("Waiting offboard mode");
  while (ros::ok() && current_state.mode != "OFFBOARD") {
    if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
      continue;
    }
    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
      ROS_INFO("Offboard should be enabled");
    }
    last_request = ros::Time::now();

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Offboard enabled");

  ROS_INFO("Arming vehicle");
  while (ros::ok() && !current_state.armed) {
    if (ros::Time::now() - last_request <= ros::Duration(5.0)) {
      continue;
    }
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("Vehicle should be armed");
    }
    last_request = ros::Time::now();

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle armed");

  while (ros::ok()) {
    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

// while (ros::ok()) {
//     if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
//         if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
//             ROS_INFO("Offboard enabled");
//         }
//         last_request = ros::Time::now();
// }


/* This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to. */
// int initialize_local_frame() {
  //set the orientation of the local reference frame
//   ROS_INFO("Initializing local coordinate system");
//   local_offset_g = 0;
//   for (int i = 1; i <= 30; i++) {
//     ros::spinOnce();
//     ros::Duration(0.1).sleep();

//     float q0 = current_pose_g.pose.pose.orientation.w;
//     float q1 = current_pose_g.pose.pose.orientation.x;
//     float q2 = current_pose_g.pose.pose.orientation.y;
//     float q3 = current_pose_g.pose.pose.orientation.z;
//     float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

//     local_offset_g += psi*(180/M_PI);

//     local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
//     local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
//     local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
//     // ROS_INFO("current heading%d: %f", i, local_offset_g/i);
//   }
//   local_offset_pose_g.x = local_offset_pose_g.x/30;
//   local_offset_pose_g.y = local_offset_pose_g.y/30;
//   local_offset_pose_g.z = local_offset_pose_g.z/30;
//   local_offset_g /= 30;
//   ROS_INFO("Coordinate offset set");
//   ROS_INFO("the X' axis is facing: %f", local_offset_g);
//   // return 0;
// // }

// // int arm() {
//   //intitialize first waypoint of mission
//   set_destination(0,0,0,0);
//   for(int i=0; i<100; i++) {
//     local_pos_pub.publish(waypoint_g);
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }
//   // arming
//   ROS_INFO("Arming drone");
//   mavros_msgs::CommandBool arm_request;
//   arm_request.request.value = true;
//   while (!current_state_g.armed && !arm_request.response.success && ros::ok()) {
//     ros::Duration(.1).sleep();
//     arming_client.call(arm_request);
//     local_pos_pub.publish(waypoint_g);
//   }
//   if (arm_request.response.success) {
//     ROS_INFO("Arming Successful");  
//     // return 0;
//   } else {
//     ROS_INFO("Arming failed with %d", arm_request.response.success);
//     return -1;  
//   }
// // }


//   wait_for_connection();
//   wait_for_start();
//   initialize_local_frame();
//   arm();
//   takeoff(3);

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

