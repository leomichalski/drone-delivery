#include <aruco_msgs/ArucoDetection.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <inttypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/LandingTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <chrono>
#include <atomic>

#include "GeographicLib/Geoid.hpp"

// state machine states
enum CURRENT_STATE_ENUM {
    STATE_STARTED,
    STATE_FCU_CONNECTED,
    STATE_GPS_CONNECTED,
    STATE_MISSION_GENERATED,
    STATE_OFFBOARD_ENABLED,
    STATE_ARMED,
    STATE_TOOK_OFF,
    STATE_AT_MISSION_STARTING_POSITION,
    STATE_DELIVERY_LOCATION_FOUND,
    STATE_DELIVERY_LOCATION_NOT_FOUND,
    STATE_RETURNED_TO_LAUNCH,
    STATE_LANDED_AT_CURRENT_POSITION,
    STATE_FAILED_SO_RETURN_TO_LAUNCH,
    STATE_FAILED_SO_LAND_AT_CURRENT_POSITION,
    STATE_FAILED_SO_HANDOVER_CONTROL,
    STATE_FAILED_SO_EXIT_PROGRAM,
    STATE_COMPLETED_SO_EXIT_PROGRAM
} CURRENT_STATE = STATE_STARTED;

// mission settings
const double TAKEOFF_HEIGHT = 30.0;  // in meters
// const double PACKAGE_DELIVERY_HEIGHT = 2.0;  // in meters
uint16_t DELIVERY_LOCATION_MARKER_ID;
// Sleeps for 30 seconds to allow someone to pick up the delivered package.
uint8_t DELIVERED_PACKAGE_PICK_UP_TIME_S = 30;  // in seconds

// TODO: determine camera center (x, y) properly OR pass it as arguments
const uint16_t CAMERA_CENTER_X = 960;
const uint16_t CAMERA_CENTER_Y = 540;

// min time interval between gps readings
uint8_t GPS_TIMEOUT_S = 30;
// wait for GPS to connect
uint8_t GPS_CONNECTION_TIMEOUT_S = 60;
uint8_t FCU_CONNECTION_TIMEOUT_S = 60;
uint8_t SET_MODE_TIMEOUT_S = 60;
uint8_t ARMING_TIMEOUT_S = 60;

std::string OFFBOARD_MODE_STR = "OFFBOARD";
// std::string PRECLAND_MODE_STR = "AUTO.PRECLAND";
std::string LAND_MODE_STR = "AUTO.LAND";

//////////////////////////////////////////////////
// TIME MEASUREMENT
//////////////////////////////////////////////////

uint64_t get_current_time_ns() {
    return std::chrono::system_clock::now().time_since_epoch().count();
}

uint64_t ns_to_s(const uint64_t stamp_ns) {
    return round(stamp_ns / 1000000000UL);
}

ros::Time ros_time_from_ns(const uint64_t stamp_ns) {
    return ros::Time(
        stamp_ns / 1000000000UL,  // t_sec
        stamp_ns % 1000000000UL  // t_nsec
    );
}

//////////////////////////////////////////////////
// GeographicLib wrapper
//////////////////////////////////////////////////

// std::shared_ptr<GeographicLib::Geoid> _egm96("egm96-5");  // WARNING: not thread safe
// GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe
std::shared_ptr<GeographicLib::Geoid> _egm96 = std::make_shared<GeographicLib::Geoid>("egm96-5");

double calc_geoid_height(double lat, double lon) {
    return (*_egm96)(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
    return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
    return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

//////////////////////////////////////////////////
// GLOBAL POSITION COMPARING
//////////////////////////////////////////////////

bool _is_same_latitude(double lat1, double lat2) {
    // 0.00005 is a 5.5 meter(s) precision
    if (std::abs(lat1 - lat2) < 0.00005) {
        return true;
    }
    return false;
}
bool _is_same_longitude(double lon1, double lon2) {
    // 0.00005 is a 5.5 meter(s) precision
    if (std::abs(lon1 - lon2) < 0.00005) {
        return true;
    }
    return false;
}
bool _is_same_altitude(double alt1, double alt2) {
    // 1.0 is 1.0 meter(s) precision
    if (std::abs(alt1 - alt2) < 1.0) {
        return true;
    }
    return false;
}

//////////////////////////////////////////////////
// MONITORING
//////////////////////////////////////////////////

geographic_msgs::GeoPoseStamped CURRENT_GLOBAL_POS;
std::mutex CURRENT_GLOBAL_POS_MUTEX;

mavros_msgs::State VEHICLE_STATE;
std::mutex VEHICLE_STATE_MUTEX;

// Publishers, subscribers, services (excluding vision node related)
ros::ServiceClient arming_client;
ros::ServiceClient setting_mode_client;
ros::ServiceClient landing_client;
ros::Publisher landing_target_publisher;
ros::Subscriber state_subscriber;
ros::Publisher global_pos_publisher;
ros::Subscriber global_pos_subscriber;
ros::Subscriber battery_subscriber;

// When using aarch64 or arm64 (Raspberry Pi OS 64-bit architecture),
//     std::atomic translates to an atomic instruction (probably LDADD).
std::atomic<bool> RECEIVED_FIRST_GLOBAL_POS(false);

void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Outside mutex to avoid hurting real time.
    // It's okay to use std::chrono::system_clock, as it won't be sent to ROS.
    ros::Time ros_time = ros_time_from_ns(get_current_time_ns());

    CURRENT_GLOBAL_POS_MUTEX.lock();
    CURRENT_GLOBAL_POS.header.stamp = ros_time;
    CURRENT_GLOBAL_POS.pose.position.latitude = msg->latitude;
    CURRENT_GLOBAL_POS.pose.position.longitude = msg->longitude;
    CURRENT_GLOBAL_POS.pose.position.altitude = ellipsoid_height_to_amsl(
        msg->latitude,
        msg->longitude,
        msg->altitude
    );
    CURRENT_GLOBAL_POS_MUTEX.unlock();

    RECEIVED_FIRST_GLOBAL_POS = true;
}

geographic_msgs::GeoPoseStamped get_current_global_pos() {
    geographic_msgs::GeoPoseStamped tmp;
    CURRENT_GLOBAL_POS_MUTEX.lock();
    tmp = CURRENT_GLOBAL_POS;
    CURRENT_GLOBAL_POS_MUTEX.unlock();
    return tmp;
}

// Maximum latitude/longitude is 180 degrees (pi radians)
double NULL_LAT = 500;
double NULL_LON = 500;
double NULL_ALT = -5000;  // much lower than sea level
bool is_vehicle_at_global_pos(double lat, double lon, double alt) {
    bool is_same = true;

    CURRENT_GLOBAL_POS_MUTEX.lock();
    if (lat != NULL_LAT)
        is_same = _is_same_latitude(lat, CURRENT_GLOBAL_POS.pose.position.latitude);

    if (lon != NULL_LON)
        is_same = is_same && _is_same_longitude(lon, CURRENT_GLOBAL_POS.pose.position.longitude);

    if (alt != NULL_ALT)
        is_same = is_same && _is_same_altitude(alt, CURRENT_GLOBAL_POS.pose.position.altitude);
    CURRENT_GLOBAL_POS_MUTEX.unlock();

    return is_same;
}


bool is_gps_connected() {
    if (!RECEIVED_FIRST_GLOBAL_POS)
        return false;
    if (ros_time_from_ns(get_current_time_ns()) - get_current_global_pos().header.stamp > ros::Duration(GPS_TIMEOUT_S)) {
        ROS_WARN("Lost GPS connection.");
        return false;
    }
    return true;
}

void vehicle_state_cb(const mavros_msgs::State::ConstPtr& msg) {
    VEHICLE_STATE_MUTEX.lock();
    VEHICLE_STATE = *msg;
    VEHICLE_STATE_MUTEX.unlock();
}

bool is_fcu_connected() {
    bool tmp;
    VEHICLE_STATE_MUTEX.lock();
    tmp = VEHICLE_STATE.connected;
    VEHICLE_STATE_MUTEX.unlock();
    return tmp;
}

bool is_vehicle_armed() {
    bool tmp;
    VEHICLE_STATE_MUTEX.lock();
    tmp = VEHICLE_STATE.armed;
    VEHICLE_STATE_MUTEX.unlock();
    return tmp;
}

std::string get_vehicle_mode() {
    std::string tmp;
    VEHICLE_STATE_MUTEX.lock();
    tmp = VEHICLE_STATE.mode;
    VEHICLE_STATE_MUTEX.unlock();
    return tmp;
}

// const uint8_t POWER_SUPPLY_HEALTH_UNKNOWN = 0;
// const uint8_t POWER_SUPPLY_HEALTH_GOOD = 1;
// const uint8_t POWER_SUPPLY_HEALTH_OVERHEAT = 2;
// const uint8_t POWER_SUPPLY_HEALTH_DEAD = 3;
// const uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
// const uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
// const uint8_t POWER_SUPPLY_HEALTH_COLD = 6;
// const uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
// const uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;

const uint8_t BATTERY_WARNING_NONE = 0;  // no battery warning
const uint8_t BATTERY_WARNING_RETURN_TO_LAUNCH = 1;  // enough battery to return to launch
const uint8_t BATTERY_WARNING_LAND_NOW = 2;  // land now, or else
std::atomic<uint8_t> CURRENT_BATTERY_WARNING(BATTERY_WARNING_NONE);
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) {
    // TODO: consider other parameters such as temperature
    // TODO: properly map "percentage" to CURRENT_BATTERY_WARNING
    float percentage = msg->percentage;
    if (msg->percentage < 0.1) {
        CURRENT_BATTERY_WARNING = BATTERY_WARNING_LAND_NOW;
    } else if (msg->percentage < 0.2) {
        CURRENT_BATTERY_WARNING = BATTERY_WARNING_RETURN_TO_LAUNCH;
    } else {
        CURRENT_BATTERY_WARNING = BATTERY_WARNING_NONE;
    }
}

//////////////////////////////////////////////////
// POSITIONING
//////////////////////////////////////////////////

void set_global_pos_destination(double lat, double lon, double alt) {
    geographic_msgs::GeoPoseStamped next_global_pos;

    // Using ros::Time (and not get_current_time_ns()) as it communicates with mavros.
    next_global_pos.header.stamp = ros::Time::now();
    // // Get current time, then convert to ros::Time.
    // ros::Time ros_time = ros_time_from_ns(get_current_time_ns());
    // next_global_pos.header.stamp = ros_time;

    next_global_pos.pose.position.latitude = lat;
    next_global_pos.pose.position.longitude = lon;
    next_global_pos.pose.position.altitude = alt;

    global_pos_publisher.publish(next_global_pos);
}

//////////////////////////////////////////////////
// VISION NODE COMMUNICATION
//////////////////////////////////////////////////

ros::Subscriber aruco_detection_subscriber;

aruco_msgs::ArucoDetection ARUCO_DETECTION;
std::mutex ARUCO_DETECTION_MUTEX;

std::atomic<bool> is_searching_delivery_location(false);
std::atomic<bool> is_delivery_location_found(false);

void aruco_detection_cb(const aruco_msgs::ArucoDetection::ConstPtr& msg) {
    ARUCO_DETECTION_MUTEX.lock();
    ARUCO_DETECTION = *msg;
    // int32_t marker_id = msg->marker_id;
    // float center_x = msg->marker_center[0];
    // float center_y = msg->marker_center[1];
    // float elapsed_time = msg->elapsed_time;
    // uint64_t image_creation_time = msg->image_creation_time;

    // TODO: receive aruco pose estimation
    ARUCO_DETECTION_MUTEX.unlock();

    if (is_searching_delivery_location && !is_delivery_location_found) {
        if (ARUCO_DETECTION.marker_id == DELIVERY_LOCATION_MARKER_ID) {
            is_delivery_location_found = true;
            ROS_INFO(
                "Found marker %d at (%f, %f).",
                ARUCO_DETECTION.marker_id,
                ARUCO_DETECTION.marker_center[0],
                ARUCO_DETECTION.marker_center[1]
            );
        }
    }
}

aruco_msgs::ArucoDetection get_aruco_detection() {
    aruco_msgs::ArucoDetection tmp;
    ARUCO_DETECTION_MUTEX.lock();
    tmp = ARUCO_DETECTION;
    ARUCO_DETECTION_MUTEX.unlock();
    return tmp;
}

// void stop_vision_node() {
//     // TODO: Send ROS message that stops the vision node.
// }

//////////////////////////////////////////////////
// STATE TRANSITIONS
//////////////////////////////////////////////////

CURRENT_STATE_ENUM wait_fcu_connection(ros::Rate rate) {
    // WAIT FOR FCU CONNECTION
    ROS_INFO("Waiting for FCU connection...");
    uint64_t start_time = ns_to_s(get_current_time_ns());
    uint64_t elapsed_time;
    while (!is_fcu_connected()) {
        // Check if there is an issue.
        elapsed_time = ns_to_s(get_current_time_ns()) - start_time;
        if (elapsed_time > FCU_CONNECTION_TIMEOUT_S) {
            ROS_INFO("FCU connection timeout, took %ld seconds", elapsed_time);
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        if (!ros::ok()) {
            ROS_INFO("ROS shutdown");
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");
    return STATE_FCU_CONNECTED;
}

CURRENT_STATE_ENUM wait_gps_connection(ros::Rate rate) {
    geographic_msgs::GeoPoseStamped current_pos;
    ROS_INFO("Waiting for GPS connection...");
    uint64_t start_time = ns_to_s(get_current_time_ns());
    uint64_t elapsed_time;
    while (!is_gps_connected()) {
        // Check if there is an issue.
        elapsed_time = ns_to_s(get_current_time_ns()) - start_time;
        if (elapsed_time > GPS_CONNECTION_TIMEOUT_S) {
            ROS_INFO("GPS connection timeout, took %ld seconds", elapsed_time);
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        if (!ros::ok()) {
            ROS_INFO("ROS shutdown");
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        ros::spinOnce();
        rate.sleep();
    }
    current_pos = get_current_global_pos();
    ROS_INFO(
        "GPS connected. Got global position: lat=%f, lon=%f, alt=%f",
        current_pos.pose.position.latitude,
        current_pos.pose.position.longitude,
        current_pos.pose.position.altitude
    );
    return STATE_GPS_CONNECTED;
}

struct global_pos_struct {
    double latitude;
    double longitude;
    double altitude;
};
std::vector<global_pos_struct> DESTINATION_LIST;


uint64_t DEST_LIST_IDX_INITIAL_POSITION;
uint64_t DEST_LIST_IDX_TAKEOFF;
uint64_t DEST_LIST_IDX_MISSION_START;
uint64_t DEST_LIST_IDX_RETURNING_TO_LAUNCH;
uint64_t DEST_LIST_IDX_LAND;
CURRENT_STATE_ENUM generate_mission(double delivery_location_lat, double delivery_location_lon, int distance_between_lines, int radius) {
    global_pos_struct dest;
    geographic_msgs::GeoPoseStamped current_pos = get_current_global_pos();
    ROS_INFO("Generated mission waypoint list:");
    // INITIAL POSITION AND GOALS
    // don't takeoff, just stay in current position
    DEST_LIST_IDX_INITIAL_POSITION = DESTINATION_LIST.size();
    dest.latitude = current_pos.pose.position.latitude;
    dest.longitude = current_pos.pose.position.longitude;
    dest.altitude = current_pos.pose.position.altitude;
    DESTINATION_LIST.push_back(dest);
    ROS_INFO("    initial_alt=%f", dest.altitude);
    ROS_INFO("    lat=%f, lon=%f, alt=%f", dest.latitude, dest.longitude, dest.altitude);
    // takeoff
    DEST_LIST_IDX_TAKEOFF = DESTINATION_LIST.size();
    dest.latitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude;
    dest.longitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude;
    dest.altitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].altitude + TAKEOFF_HEIGHT;
    DESTINATION_LIST.push_back(dest);
    ROS_INFO("    lat=%f, lon=%f, alt=%f", dest.latitude, dest.longitude, dest.altitude);

    // Run script that generates the mission waypoints needed to search for the aruco marker
    std::string waypoints_file(getenv("XDG_RUNTIME_DIR"));
    waypoints_file.append("/waypoints.txt");

    std::string cmd = (
        std::string("rosrun aruco_hunter scripts/generate_mission_waypoints.py ") +
        " --mav-location-lat=" + std::to_string(DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude) +
        " --mav-location-lon=" + std::to_string(DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude) +
        " --delivery-location-lat=" + std::to_string(delivery_location_lat) +
        " --delivery-location-lon=" + std::to_string(delivery_location_lon) +
        " --distance-between-lines=" + std::to_string(distance_between_lines) +
        " --radius=" + std::to_string(radius) +
        " > " + waypoints_file
    );
    system(cmd.c_str());

    // Read mission waypoints from a file.
    std::ifstream source;

    // The mission starting position is the first waypoint in the file.
    DEST_LIST_IDX_MISSION_START = DESTINATION_LIST.size() + 1;

    source.open(waypoints_file, std::ios_base::in);
    for (std::string line; std::getline(source, line);) {
        std::istringstream in(line);  // make a stream for the line itself
        in >> dest.latitude >> dest.longitude;
        DESTINATION_LIST.push_back(dest);
        ROS_INFO("    lat=%f, lon=%f, alt=%f", dest.latitude, dest.longitude, dest.altitude);
    }

    // return to launch
    DEST_LIST_IDX_RETURNING_TO_LAUNCH = DESTINATION_LIST.size();
    dest.latitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude;
    dest.longitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude;
    dest.altitude = DESTINATION_LIST[DEST_LIST_IDX_TAKEOFF].altitude;
    DESTINATION_LIST.push_back(dest);
    ROS_INFO("    lat=%f, lon=%f, alt=%f", dest.latitude, dest.longitude, dest.altitude);
    // land
    DEST_LIST_IDX_LAND = DESTINATION_LIST.size();
    dest.latitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude;
    dest.longitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude;
    dest.altitude = DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].altitude;
    DESTINATION_LIST.push_back(dest);
    ROS_INFO("    lat=%f, lon=%f, alt=%f", dest.latitude, dest.longitude, dest.altitude);

    return STATE_MISSION_GENERATED;
}

CURRENT_STATE_ENUM set_offboard_mode(ros::Rate rate) {
    ROS_INFO("Waiting offboard mode");
    mavros_msgs::SetMode set_mode;
    set_mode.request.base_mode = 0;
    set_mode.request.custom_mode = OFFBOARD_MODE_STR;

    uint64_t last_request_time = 0;
    uint64_t start_time = ns_to_s(get_current_time_ns());
    uint64_t elapsed_time;
    while (get_vehicle_mode() != OFFBOARD_MODE_STR) {
        // Check if there is an issue.
        elapsed_time = ns_to_s(get_current_time_ns()) - start_time;
        if (elapsed_time > SET_MODE_TIMEOUT_S) {
            ROS_INFO("Timeout while setting OFFBOARD mode, took %ld seconds:", elapsed_time);
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        if (!ros::ok()) {
            ROS_INFO("ROS shutdown");
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        // Send a destination to the FCU, otherwise it will not work.
        set_global_pos_destination(
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude,
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude,
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].altitude
        );
        // Try setting the mode. If it fails, try again in 5 seconds.
        if ((last_request_time == 0) || (ns_to_s(get_current_time_ns()) - last_request_time > 5)) {
            if (setting_mode_client.call(set_mode) && set_mode.response.mode_sent) {
                // The message was sent successfully, but it does not mean that the mode was set successfully.
                ROS_INFO("Sent 'set mode to OFFBOARD' message");
            } else {
                ROS_INFO("Failed to send 'set mode to OFFBOARD' message, trying again in 5 seconds");
            }
            last_request_time = ns_to_s(get_current_time_ns());
        }
        // Ensure loop rate.
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle mode set to OFFBOARD.");
    return STATE_OFFBOARD_ENABLED;
}

CURRENT_STATE_ENUM arm(ros::Rate rate) {
    ROS_INFO("Arming vehicle");

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    uint64_t last_request_time = 0;
    uint64_t start_time = ns_to_s(get_current_time_ns());
    uint64_t elapsed_time;
    while (!is_vehicle_armed()) {
        // Check if there is an issue.
        elapsed_time = ns_to_s(get_current_time_ns()) - start_time;
        if (elapsed_time > ARMING_TIMEOUT_S) {
            ROS_INFO("Timeout while arming vehicle, took %ld seconds", elapsed_time);
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        if (!ros::ok()) {
            ROS_INFO("ROS shutdown");
            return STATE_FAILED_SO_EXIT_PROGRAM;
        }
        // Try arming. If it fails, try again in 5 seconds.
        ROS_INFO("Waiting arm...");
        if ((last_request_time == 0) || (ns_to_s(get_current_time_ns()) - last_request_time > 5)) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                // The message was sent successfully, but it does not mean that the vehicle was armed successfully.
                ROS_INFO("Sent arming message");
            } else {
                ROS_INFO("Failed to sent arming message, trying again in 5 seconds");
            }
            last_request_time = ns_to_s(get_current_time_ns());
        }
        // Send a destination to the FCU, otherwise it will not work.
        set_global_pos_destination(
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].latitude,
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].longitude,
            DESTINATION_LIST[DEST_LIST_IDX_INITIAL_POSITION].altitude
        );
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed");
    return STATE_ARMED;
}

CURRENT_STATE_ENUM takeoff(ros::Rate rate) {
    ROS_INFO("Taking off...");
    geographic_msgs::GeoPoseStamped current_pos = get_current_global_pos();
    double target_lat = current_pos.pose.position.latitude;
    double target_lon = current_pos.pose.position.longitude;
    while (get_current_global_pos().pose.position.altitude < (0.95 * DESTINATION_LIST[DEST_LIST_IDX_TAKEOFF].altitude)) {
        if (!is_fcu_connected() || !is_gps_connected() || (get_vehicle_mode() != OFFBOARD_MODE_STR) || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS, current mode or ROS.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_LAND_NOW) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            return STATE_FAILED_SO_LAND_AT_CURRENT_POSITION;
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        set_global_pos_destination(
            target_lat,
            target_lon,
            DESTINATION_LIST[DEST_LIST_IDX_TAKEOFF].altitude
        );
        ros::spinOnce();
        rate.sleep();
    }
    return STATE_TOOK_OFF;
}

CURRENT_STATE_ENUM go_to_mission_starting_position(ros::Rate rate) {
    ROS_INFO("Going to mission starting position...");
    while (!is_vehicle_at_global_pos(DESTINATION_LIST[DEST_LIST_IDX_MISSION_START].latitude,
                                     DESTINATION_LIST[DEST_LIST_IDX_MISSION_START].longitude,
                                     NULL_ALT)) {
        if (!is_fcu_connected() || !is_gps_connected() || (get_vehicle_mode() != OFFBOARD_MODE_STR) || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS, current mode or ROS.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_LAND_NOW) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            return STATE_FAILED_SO_LAND_AT_CURRENT_POSITION;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_RETURN_TO_LAUNCH) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            return STATE_FAILED_SO_RETURN_TO_LAUNCH;
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        set_global_pos_destination(
            DESTINATION_LIST[DEST_LIST_IDX_MISSION_START].latitude,
            DESTINATION_LIST[DEST_LIST_IDX_MISSION_START].longitude,
            DESTINATION_LIST[DEST_LIST_IDX_MISSION_START].altitude
        );
        ros::spinOnce();
        rate.sleep();
    }
    return STATE_AT_MISSION_STARTING_POSITION;
}

CURRENT_STATE_ENUM search_for_delivery_location(ros::Rate rate) {
    ROS_INFO("Going to mission starting position...");
    is_searching_delivery_location = true;
    uint64_t current_dest_idx = DEST_LIST_IDX_MISSION_START + 1;
    while (!is_delivery_location_found) {
        // Check if there is an issue.
        if (!is_fcu_connected() || !is_gps_connected() || (get_vehicle_mode() != OFFBOARD_MODE_STR) || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS, current mode or ROS.");
            is_searching_delivery_location = false;
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_LAND_NOW) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            is_searching_delivery_location = false;
            return STATE_FAILED_SO_LAND_AT_CURRENT_POSITION;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_RETURN_TO_LAUNCH) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            is_searching_delivery_location = false;
            return STATE_FAILED_SO_RETURN_TO_LAUNCH;
        }
        // Update destination if current destination reached.
        if (is_vehicle_at_global_pos(DESTINATION_LIST[current_dest_idx].latitude,
                                     DESTINATION_LIST[current_dest_idx].longitude,
                                     NULL_ALT)) {
            // Update current destination index.
            current_dest_idx++;
            if (current_dest_idx >= DESTINATION_LIST.size()) {
                ROS_INFO("All destinations reached, no ArUco tag found");
                is_searching_delivery_location = false;
                return STATE_DELIVERY_LOCATION_NOT_FOUND;
            }
            // Log current destination.
            ROS_INFO(
                "UAV heading to lat=%f, lon=%f, alt=%f",
                DESTINATION_LIST[current_dest_idx].latitude,
                DESTINATION_LIST[current_dest_idx].longitude,
                DESTINATION_LIST[current_dest_idx].altitude
            );
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        set_global_pos_destination(
            DESTINATION_LIST[current_dest_idx].latitude,
            DESTINATION_LIST[current_dest_idx].longitude,
            DESTINATION_LIST[current_dest_idx].altitude
        );
        ros::spinOnce();
        rate.sleep();
    }
    is_searching_delivery_location = false;
    return STATE_DELIVERY_LOCATION_FOUND;
}

// WARNING: Only allows a transition from the OFFBOARD mode to the AUTO.LAND mode!!
//     Assume manual control otherwise.
CURRENT_STATE_ENUM land_at_current_position(ros::Rate rate) {
    geographic_msgs::GeoPoseStamped current_pos;
    uint64_t last_still_landing_log_time = 0;
    uint64_t last_request_time = 0;
    // Empty command means land at current location.
    mavros_msgs::CommandTOL landing_cmd;

    ROS_INFO("Exiting offboard mode, entering land mode...");
    while (get_vehicle_mode() != LAND_MODE_STR) {
        // Check if there is an issue.
        if ((get_vehicle_mode() != OFFBOARD_MODE_STR) && (get_vehicle_mode() != LAND_MODE_STR)) {
            ROS_INFO("Issue with current mode.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        }
        if (!is_fcu_connected() || !is_gps_connected() || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS or ROS.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        }
        // Exit OFFBOARD mode to enter the LAND mode. If it fails, try again in 1 second.
        ROS_INFO("Waiting land mode to be enabled...");
        if ((last_request_time == 0) || (ns_to_s(get_current_time_ns()) - last_request_time > 1)) {
            if (landing_client.call(landing_cmd) && landing_cmd.response.success) {
                // The message was sent successfully, but it does not mean that the vehicle was armed successfully.
                ROS_INFO("Sent landing message");
            } else {
                ROS_INFO("Failed to sent landing message, trying again in 1 second");
            }
            last_request_time = ns_to_s(get_current_time_ns());
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        ros::spinOnce();
        rate.sleep();
    }

    current_pos = get_current_global_pos();
    ROS_INFO(
        "Landing right now at lat=%f, lon=%f, alt=%f...", 
        current_pos.pose.position.latitude,
        current_pos.pose.position.longitude,
        current_pos.pose.position.altitude
    );

    // Wait for the vehicle to land. This is done by checking if the vehicle is armed.
    while (is_vehicle_armed()) {
        // Check if there is an issue.
        if (!is_fcu_connected() || !is_gps_connected() || (get_vehicle_mode() != LAND_MODE_STR) || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS, current mode or ROS.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        }
        // Every second, log a message that the vehicle is still landing.
        if ((last_still_landing_log_time == 0) || (ns_to_s(get_current_time_ns()) - last_still_landing_log_time > 1)) {
            ROS_INFO("Vehicle is still landing...");
            last_still_landing_log_time = ns_to_s(get_current_time_ns());
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        ros::spinOnce();
        rate.sleep();
    }
    current_pos = get_current_global_pos();
    ROS_INFO(
        "Landed successfully at lat=%f, lon=%f, alt=%f...", 
        current_pos.pose.position.latitude,
        current_pos.pose.position.longitude,
        current_pos.pose.position.altitude
    );

    return STATE_LANDED_AT_CURRENT_POSITION;
}

/*
// WARNING: Only allows a transition from the OFFBOARD mode to the AUTO.PRECLAND mode!!
//     Assume manual control otherwise.
CURRENT_STATE_ENUM hover_while_setting_precision_landing_mode(ros::Rate rate) {
    ROS_INFO("Hovering while setting precision landing mode...");

    mavros_msgs::SetMode set_mode;
    set_mode.request.base_mode = 4;  // PX4_CUSTOM_MAIN_MODE_AUTO 
    set_mode.request.custom_mode = PRECLAND_MODE_STR;  // PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND

    uint64_t last_request_time = 0;

    geographic_msgs::GeoPoseStamped current_pos = get_current_global_pos();
    double target_lat = current_pos.pose.position.latitude;
    double target_lon = current_pos.pose.position.longitude;
    double target_alt = DESTINATION_LIST[DEST_LIST_IDX_TAKEOFF].altitude;

    while (get_vehicle_mode() != PRECLAND_MODE_STR) {
        // Check if there is an issue.
        if ((get_vehicle_mode() != OFFBOARD_MODE_STR) && (get_vehicle_mode() != PRECLAND_MODE_STR)) {
            ROS_INFO("Issue with current mode.");
            is_searching_delivery_location = false;
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        }
        if (!is_fcu_connected() || !is_gps_connected() || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS or ROS.");
            is_searching_delivery_location = false;
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        }
        // Hover.
        set_global_pos_destination(
            target_lat,
            target_lon,
            target_alt
        );
        // Try setting the mode. If it fails, try again in 5 seconds.
        if ((last_request_time == 0) || (ns_to_s(get_current_time_ns()) - last_request_time > 5)) {
            if (setting_mode_client.call(set_mode) && set_mode.response.mode_sent) {
                // The message was sent successfully, but it does not mean that the mode was set successfully.
                ROS_INFO("Sent 'set mode to AUTO.PRECLAND' message");
            } else {
                ROS_INFO("Failed to send 'set mode to AUTO.PRECLAND' message, trying again in 5 seconds");
            }
            last_request_time = ns_to_s(get_current_time_ns());
        }
        // Ensure loop rate.
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle mode set to AUTO.PRECLAND.");
    return STATE_PRECISION_LANDING_MODE_ENABLED;
}
*/

CURRENT_STATE_ENUM return_to_launch(ros::Rate rate) {
    ROS_INFO("Returning to launch position...");
    while (!is_vehicle_at_global_pos(DESTINATION_LIST[DEST_LIST_IDX_RETURNING_TO_LAUNCH].latitude,
                                     DESTINATION_LIST[DEST_LIST_IDX_RETURNING_TO_LAUNCH].longitude,
                                     NULL_ALT)) {
        if (!is_fcu_connected() || !is_gps_connected() || (get_vehicle_mode() != OFFBOARD_MODE_STR) || !ros::ok()) {
            ROS_INFO("Issue with FCU, GPS, current mode or ROS.");
            return STATE_FAILED_SO_HANDOVER_CONTROL;
        } else if (CURRENT_BATTERY_WARNING == BATTERY_WARNING_LAND_NOW) {
            ROS_INFO("Battery warning level is %d", CURRENT_BATTERY_WARNING.load());
            return STATE_FAILED_SO_LAND_AT_CURRENT_POSITION;
        }
        // If the vehicle gets outside the geofence (defined in qgroundcontrol),
        //     the motors will be turned off by px4.
        set_global_pos_destination(
            DESTINATION_LIST[DEST_LIST_IDX_RETURNING_TO_LAUNCH].latitude,
            DESTINATION_LIST[DEST_LIST_IDX_RETURNING_TO_LAUNCH].longitude,
            DESTINATION_LIST[DEST_LIST_IDX_RETURNING_TO_LAUNCH].altitude
        );
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Returned to launch latitude and longitude.");
    return STATE_RETURNED_TO_LAUNCH;
}

CURRENT_STATE_ENUM hand_over_control() {
    ROS_INFO("URGENT WARNING: Assume manual control from the FCU. Companion computer flight control is OFF, and the mission CANNOT be resumed. It must be restarted.");
    exit(-1);  // TODO: proper exit code
}

//////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////

int main(int argc, char** argv) {

    int exit_code = -1;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    double delivery_location_lat, delivery_location_lon;
    int distance_between_lines, radius;

    nh.getParam("/main/delivery_location_lat", delivery_location_lat);
    nh.getParam("/main/delivery_location_lon", delivery_location_lon);
    nh.getParam("/main/distance_between_lines", distance_between_lines);
    nh.getParam("/main/radius", radius);

    // TODO: should be in .launch
    DELIVERY_LOCATION_MARKER_ID = 4;

    aruco_detection_subscriber = nh.subscribe<aruco_msgs::ArucoDetection>("/aruco_detection", 1, aruco_detection_cb);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setting_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    // landing_target_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/landing_target/pose", 1);
    state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state", 1, vehicle_state_cb);
    global_pos_publisher = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 1);
    global_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_position_cb);
    battery_subscriber = nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 1, battery_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);
    // ros::Rate rate(20.0);

    while (ros::ok()) {
        if (CURRENT_STATE == STATE_STARTED) {
            // Take off, go to mission starting location and search for an aruco marker.
            CURRENT_STATE = wait_fcu_connection(rate);
            if (CURRENT_STATE != STATE_FCU_CONNECTED)
                continue;
            CURRENT_STATE = wait_gps_connection(rate);
            if (CURRENT_STATE != STATE_GPS_CONNECTED)
                continue;
            CURRENT_STATE = generate_mission(delivery_location_lat, delivery_location_lon, distance_between_lines, radius);
            if (CURRENT_STATE != STATE_MISSION_GENERATED)
                continue;
            CURRENT_STATE = set_offboard_mode(rate);
            if (CURRENT_STATE != STATE_OFFBOARD_ENABLED)
                continue;
            CURRENT_STATE = arm(rate);
            if (CURRENT_STATE != STATE_ARMED)
                continue;
            CURRENT_STATE = takeoff(rate);
            if (CURRENT_STATE != STATE_TOOK_OFF)
                continue;
            CURRENT_STATE = go_to_mission_starting_position(rate);
            if (CURRENT_STATE != STATE_AT_MISSION_STARTING_POSITION)
                continue;
            CURRENT_STATE = search_for_delivery_location(rate);
            if (CURRENT_STATE != STATE_DELIVERY_LOCATION_FOUND)
                continue;
            // If the delivery location is found, land at the delivery location.
            CURRENT_STATE = land_at_current_position(rate);
            if (CURRENT_STATE != STATE_LANDED_AT_CURRENT_POSITION)
                continue;
            // Deliver the package.
            ROS_INFO("Sleeping for 30 seconds to allow someone to pick up the delivered package.");
            sleep(DELIVERED_PACKAGE_PICK_UP_TIME_S);
            // Change mode from AUTO.LAND back to OFFBOARD.
            CURRENT_STATE = set_offboard_mode(rate);
            if (CURRENT_STATE != STATE_OFFBOARD_ENABLED)
                continue;
            // Arm the vehicle (automatically disarmed after landing).
            CURRENT_STATE = arm(rate);
            if (CURRENT_STATE != STATE_ARMED)
                continue;
            // Take off again before returning to launch.
            CURRENT_STATE = takeoff(rate);
            if (CURRENT_STATE != STATE_TOOK_OFF)
                continue;
            // Return to launch without package.
            CURRENT_STATE = return_to_launch(rate);
            if (CURRENT_STATE != STATE_RETURNED_TO_LAUNCH)
                continue;
            CURRENT_STATE = land_at_current_position(rate);
            if (CURRENT_STATE != STATE_LANDED_AT_CURRENT_POSITION)
                continue;
            CURRENT_STATE = STATE_COMPLETED_SO_EXIT_PROGRAM;
        } else if ((CURRENT_STATE == STATE_DELIVERY_LOCATION_NOT_FOUND) || (CURRENT_STATE == STATE_FAILED_SO_RETURN_TO_LAUNCH)) {
            // Return to launch with package.
            CURRENT_STATE = return_to_launch(rate);
            if (CURRENT_STATE != STATE_RETURNED_TO_LAUNCH)
                continue;
            CURRENT_STATE = land_at_current_position(rate);
            if (CURRENT_STATE != STATE_LANDED_AT_CURRENT_POSITION)
                continue;
            CURRENT_STATE = STATE_COMPLETED_SO_EXIT_PROGRAM;
        } else if (CURRENT_STATE == STATE_FAILED_SO_LAND_AT_CURRENT_POSITION) {
            // Land immediately at current position code.
            CURRENT_STATE = land_at_current_position(rate);
        } else if (CURRENT_STATE == STATE_FAILED_SO_HANDOVER_CONTROL) {
            // Assume manual control.
            CURRENT_STATE = hand_over_control();
        } else if (CURRENT_STATE == STATE_FAILED_SO_EXIT_PROGRAM) {
            // Exit program with fail status.
            // stop_vision_node();
            ROS_INFO("Program exited with an error.");  // TODO: proper exit message
            exit_code = -1;  // TODO: proper exit code
            break;
        } else if (CURRENT_STATE == STATE_COMPLETED_SO_EXIT_PROGRAM) {
            // EXIT PROGRAM WITH SUCCESS STATUS
            // stop_vision_node();
            ROS_INFO("Program completed successfully.");
            exit_code = 0;
            break;
        } else {
            // HANDLE UNKNOWN STATE (SHOULD NEVER HAPPEN)
            ROS_INFO("State %d doesn't exist.", CURRENT_STATE);
            CURRENT_STATE = STATE_FAILED_SO_HANDOVER_CONTROL;
        }
    }
    ROS_INFO("Companion computer flight control is OFF, and the mission CANNOT be resumed. It must be restarted.");
    return exit_code;
}
