# python3 scripts/package_delivery_flight_planning.py > flight/aruco_hunter/src/waypoints.txt
# it would be probably be better to solve this problem using analytical geometry, but I had a lot of fixed-point arithmetic trouble with it so I'm using a greedy algorithm
import numpy as np
import gmpy2

gmpy2.get_context().precision = 50

PI = gmpy2.const_pi()
TWO_PI = 2 * PI
THREE_PI = 3 * PI
EARTH_RADIUS = gmpy2.mpfr(6371000)
# Multiply the degrees of separation of longitude and latitude by 111,139 to get the corresponding linear distances in meters
METERS_TO_SEP = 1 / 111139
SEP_TO_METERS = 111139


def abs(x):
    return -x if gmpy2.is_signed(x) else x


def degrees_to_radians(deg):
    return deg * PI / 180.0


def radians_to_degrees(rad):
    return rad * 180.0 / PI


def circumference_point(center_point, radius, angle):
    """
    Returns a point that is located at a given distance (radius) and at a given angle (angle in degrees) from a reference point (center_point).

    center_point: [latitude, longitude] of the reference point
    radius: distance from the reference point in meters
    angle: angle in degrees from the reference point
    """
    center_point = gmpy2.mpfr(center_point[0]), gmpy2.mpfr(center_point[1])
    radius = gmpy2.mpfr(radius)
    angle = gmpy2.mpfr(angle)

    lat_sin = gmpy2.sin(degrees_to_radians(center_point[0]))
    lat_cos = gmpy2.cos(degrees_to_radians(center_point[0]))

    # random bearing(direction out 360 degrees)
    bearing = angle * TWO_PI / 360
    bearing_sin = gmpy2.sin(bearing)
    bearing_cos = gmpy2.cos(bearing)

    # theta is the approximated angular distance
    theta = radius / EARTH_RADIUS
    theta_sin = gmpy2.sin(theta)
    theta_cos = gmpy2.cos(theta)

    r_latitude = gmpy2.asin(lat_sin * theta_cos + lat_cos * theta_sin * bearing_cos)

    r_longitude = degrees_to_radians(center_point[1]) + gmpy2.atan2(
        bearing_sin * theta_sin * lat_cos,
        theta_cos - lat_sin * gmpy2.sin(r_latitude)
    )

    # normalize longitude L such that - PI < L < +PI
    r_longitude = ((r_longitude + THREE_PI) % TWO_PI) - PI

    return [radians_to_degrees(r_latitude), radians_to_degrees(r_longitude)]



def closest_wp_idx(reference, wp_list):
    closest_wp = wp_list[0]
    closest_dist = (reference[0] - closest_wp[0])**2 + (reference[1] - closest_wp[1])**2
    for curr_wp in wp_list[1:]:
        curr_dist = (reference[0] - curr_wp[0])**2 + (reference[1] - curr_wp[1])**2
        if closest_dist > curr_dist:
            closest_wp = curr_wp
            closest_dist = curr_dist
    return wp_list.index(closest_wp)


def distance_point_to_line(point, line):
    [x0, y0], [x1, y1] = line
    return abs((y1 - y0) * point[0] - (x1 - x0) * point[1] + x1 * y0 - y1 * x0) / gmpy2.sqrt((y1 - y0) ** 2 + (x1 - x0) ** 2)

# selecionar e ordenar os waypoints
def select_waypoints(wp_list, initial_wp_idx):
    selected_wp_list = [MAV_LOCATION, wp_list[initial_wp_idx]]
    left_adjust = -1
    right_adjust = 1
    right_first = True

    for _ in range((len(wp_list) // 2) - 1):
        left_idx = (initial_wp_idx + left_adjust) % 360
        right_idx = (initial_wp_idx + right_adjust) % 360

        left_wp = wp_list[left_idx]
        right_wp = wp_list[right_idx]

        dist_from_prev_wp = distance_point_to_line(
            point=selected_wp_list[-1],
            line=[left_wp, right_wp]
        )

        if dist_from_prev_wp > DISTANCE_BETWEEN_LINES * METERS_TO_SEP:
            if right_first:
                selected_wp_list.append(right_wp)
                selected_wp_list.append(left_wp)
            else:
                selected_wp_list.append(left_wp)
                selected_wp_list.append(right_wp)
            right_first = not right_first

        left_adjust -= 1
        right_adjust += 1
    return selected_wp_list


if __name__ == '__main__':

    MAV_LOCATION = [gmpy2.mpfr(52.171974), gmpy2.mpfr(4.417091)]
    DELIVERY_LOCATION = [gmpy2.mpfr(52.169916), gmpy2.mpfr(4.415763)]
    DISTANCE_BETWEEN_LINES = gmpy2.mpfr(10)  # in meters
    RADIUS = gmpy2.mpfr(111)  # in meters

    # gerar 360 pontos na circunferencia
    tmp_wp_list = []
    for degs in range(0, 360):
        wp = circumference_point(
            center_point=DELIVERY_LOCATION,
            radius=RADIUS,
            angle=degs
        )
        tmp_wp_list.append([x, y])

    # ver qual eh o ponto mais proximo do MAV
    initial_wp_idx = closest_wp_idx(MAV_LOCATION, tmp_wp_list)

    selected_wp_list = select_waypoints(tmp_wp_list, initial_wp_idx)
    for wp in selected_wp_list:
        print("%.9f %.9f" % (wp[0], wp[1]))
