# python3 scripts/package_delivery_flight_planning.py > flight/aruco_hunter/src/waypoints.txt
import numpy as np
from decimal import Decimal, getcontext


# STUFF THE PYTHON LIB "DECIMAL" REQUIRES

def pi():
    """Compute Pi to the current precision.

    >>> print(pi())
    3.141592653589793238462643383

    """
    getcontext().prec += 2  # extra digits for intermediate steps
    three = Decimal(3)      # substitute "three=3.0" for regular floats
    lasts, t, s, n, na, d, da = 0, three, 3, 1, 0, 0, 24
    while s != lasts:
        lasts = s
        n, na = n+na, na+8
        d, da = d+da, da+32
        t = (t * n) / d
        s += t
    getcontext().prec -= 2
    return +s               # unary plus applies the new precision


def exp(x):
    """Return e raised to the power of x.  Result type matches input type.

    >>> print(exp(Decimal(1)))
    2.718281828459045235360287471
    >>> print(exp(Decimal(2)))
    7.389056098930650227230427461
    >>> print(exp(2.0))
    7.38905609893
    >>> print(exp(2+0j))
    (7.38905609893+0j)

    """
    getcontext().prec += 2
    i, lasts, s, fact, num = 0, 0, 1, 1, 1
    while s != lasts:
        lasts = s
        i += 1
        fact *= i
        num *= x
        s += num / fact
    getcontext().prec -= 2
    return +s


def cos(x):
    """Return the cosine of x as measured in radians.

    The Taylor series approximation works best for a small value of x.
    For larger values, first compute x = x % (2 * pi).

    >>> print(cos(Decimal('0.5')))
    0.8775825618903727161162815826
    >>> print(cos(0.5))
    0.87758256189
    >>> print(cos(0.5+0j))
    (0.87758256189+0j)

    """
    getcontext().prec += 2
    i, lasts, s, fact, num, sign = 0, 0, 1, 1, 1, 1
    while s != lasts:
        lasts = s
        i += 2
        fact *= i * (i-1)
        num *= x * x
        sign *= -1
        s += num / fact * sign
    getcontext().prec -= 2
    return +s


def sin(x):
    """Return the sine of x as measured in radians.

    The Taylor series approximation works best for a small value of x.
    For larger values, first compute x = x % (2 * pi).

    >>> print(sin(Decimal('0.5')))
    0.4794255386042030002732879352
    >>> print(sin(0.5))
    0.479425538604
    >>> print(sin(0.5+0j))
    (0.479425538604+0j)

    """
    getcontext().prec += 2
    i, lasts, s, fact, num, sign = 1, 0, x, 1, x, 1
    while s != lasts:
        lasts = s
        i += 2
        fact *= i * (i-1)
        num *= x * x
        sign *= -1
        s += num / fact * sign
    getcontext().prec -= 2
    return +s

# AAAAAAAAAAAAAAAAAAAAAAA

# 
def closest_wp_idx(wp, wp_list):
    wp = [float(wp[0]), float(wp[1])]
    wp_list = np.asarray([[float(wp[0]), float(wp[1])] for wp in wp_list])
    dist = np.sum((wp_list - wp)**2, axis=1)
    return np.argmin(dist)


# distancia entre um ponto e uma linha
def distance_point_to_line(point, line):
    [x0, y0], [x1, y1] = line
    return abs((y1 - y0) * point[0] - (x1 - x0) * point[1] + x1 * y0 - y1 * x0) / ((y1 - y0) ** 2 + (x1 - x0) ** 2).sqrt()


# selecionar e ordenar os waypoints
def select_waypoints(wp_list, initial_wp_idx):
    selected_wp_list = [MAV_LOCATION, wp_list[initial_wp_idx]]
    left_adjust = -1
    right_adjust = 1
    right_first = True

    for _ in range(len(wp_list)//2):
        left_idx = (initial_wp_idx + left_adjust) % 360
        right_idx = (initial_wp_idx + right_adjust) % 360

        left_wp = wp_list[left_idx]
        right_wp = wp_list[right_idx]

        try:
            dist_from_prev_wp = distance_point_to_line(
                point=selected_wp_list[-1],
                line=[left_wp, right_wp]
            )
        except:
            left_adjust -= 1
            right_adjust += 1
            continue

        if dist_from_prev_wp > DISTANCE_BETWEEN_LINES:
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

    MAV_LOCATION = [Decimal(52.171974), Decimal(4.417091)]
    DELIVERY_LOCATION = [Decimal(52.169916), Decimal(4.415763)]
    DISTANCE_BETWEEN_LINES = Decimal(0.0001)  # equivalent to 11.1 meters
    RADIUS = Decimal(0.001)  # equivalent to 111 meters

    # gerar 360 pontos na circunferencia
    tmp_wp_list = []
    for degs in range(0, 360):
        rads = Decimal(degs) * pi() / Decimal(180)
        x = cos(rads)*RADIUS + DELIVERY_LOCATION[0]
        y = sin(rads)*RADIUS + DELIVERY_LOCATION[1]
        tmp_wp_list.append([x, y])

    # ver qual eh o ponto mais proximo do MAV
    initial_wp_idx = closest_wp_idx(wp=MAV_LOCATION, wp_list=tmp_wp_list)

    selected_wp_list = select_waypoints(tmp_wp_list, initial_wp_idx)
    for wp in selected_wp_list:
        print("%.9f %.9f" % (wp[0], wp[1]))
