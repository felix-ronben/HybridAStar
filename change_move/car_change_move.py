"""

Car model for Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""


from xml.etree.ElementTree import PI
import matplotlib.pyplot as plt
from math import radians, sqrt, cos, sin, tan, pi, atan

WB = 3.  # rear to front wheel
W = 2.  # width of car
LF = 3.3  # distance from rear to vehicle front end
LB = 1.0  # distance from rear to vehicle back end
MAX_STEER = 0.6  # [rad] maximum steering angle

WBUBBLE_DIST = (LF - LB) / 2.0
WBUBBLE_R = sqrt(((LF + LB) / 2.0)**2 + 1)

# vehicle rectangle verticles
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W / 2, -W / 2, -W / 2, W / 2, W / 2]


def check_car_collision(xlist, ylist, yawlist, ox, oy, kdtree):
    for x, y, yaw in zip(xlist, ylist, yawlist):
        cx = x + WBUBBLE_DIST * cos(yaw)
        cy = y + WBUBBLE_DIST * sin(yaw)

        ids = kdtree.search_in_distance([cx, cy], WBUBBLE_R)

        if not ids:
            continue

        if not rectangle_check(x, y, yaw,
                               [ox[i] for i in ids], [oy[i] for i in ids]):
            return False  # collision

    return True  # no collision


def rectangle_check(x, y, yaw, ox, oy):
    # transform obstacles to base link frame
    c, s = cos(-yaw), sin(-yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        rx = c * tx - s * ty
        ry = s * tx + c * ty

        if not (rx > LF or rx < -LB or ry > W / 2.0 or ry < -W / 2.0):
            return False  # no collision

    return True  # collision


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)
        # plt.plot(x, y)


def plot_car(x, y, yaw):
    car_color = '-k'
    c, s = cos(yaw), sin(yaw)

    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        tx = c * rx - s * ry + x
        ty = s * rx + c * ry + y
        car_outline_x.append(tx)
        car_outline_y.append(ty)

    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y, car_color)


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi


def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw += pi_2_pi(distance * tan(steer) / L)  # distance/2

    return x, y, yaw


def new_move(x, y, yaw, distance, steer, L=WB):
    if steer != 0:
        r = L / tan(steer)
        theta = pi_2_pi(distance / r)
        tmp_x = r * sin(theta)
        tmp_y = r - r*cos(theta)
        new_r = sqrt(tmp_x**2 + tmp_y**2)
        theta0 = yaw + atan(tmp_y/tmp_x)
        x += new_r * cos(theta0)
        y += new_r * sin(theta0)
        yaw += theta
    else:
        x += distance*cos(yaw)
        y += distance*sin(yaw)

    return x, y, yaw

    
def spr_move1(x, y, yaw, distance, steer, len_s, L=WB, radi=None):
    ls, l = len_s, distance
    r = L / tan(steer) if radi is None else radi
    c = l*(1-(l**4)/(90*r**2*ls**2)+(l**8)/(22680*r**4*ls**4))  #-(79*l**12)/(2043241200*r**6*ls**6))  # 弦长
    theta = (l**2)/(r*ls)*(1/6-(l**4)/(2835*r**2*ls**2)-(l**8)/(467775*r**4*ls**4))  #-(23*l**12)/(1915538625*r**6*ls**6))  # 弦切角
    beta = (l**2)/(2*r*ls)  # 切线角
    theta0 = yaw + theta
    x += c * cos(theta0)
    y += c * sin(theta0)
    yaw += beta

    return x, y, yaw


def spr_move2(x, y, yaw, distance, steer, len_s, L=WB):
    ls, l = len_s, len_s
    r = L / tan(steer)
    c = l*(1-(l**2)/(90*r**2)+(l**4)/(22680*r**4))  #-(79*l**6)/(2043241200*r**6))  # 弦长
    theta = (l)/(r)*(1/6-(l**2)/(2835*r**2)-(l**4)/(467775*r**4))  #-(23*l**6)/(1915538625*r**6))  # 弦切角
    beta = (l)/(2*r)  # 切线角
    theta0 = yaw + (beta-theta)
    x_final = x + c * cos(theta0)
    y_final = y + c * sin(theta0)
    yaw_final = pi_2_pi(yaw + beta)
    
    ls, l = len_s, len_s - distance
    c = l*(1-(l**4)/(90*r**2*ls**2)+(l**8)/(22680*r**4*ls**4))  #-(79*l**12)/(2043241200*r**6*ls**6))  # 弦长
    theta = (l**2)/(r*ls)*(1/6-(l**4)/(2835*r**2*ls**2)-(l**8)/(467775*r**4*ls**4))  #-(23*l**12)/(1915538625*r**6*ls**6))  # 弦切角
    beta = (l**2)/(2*r*ls)  # 切线角
    theta0 = yaw_final + pi - theta
    x = x_final + c * cos(theta0)
    y = y_final + c * sin(theta0)
    yaw = yaw_final - beta

    return x, y, yaw


def r_move(x, y, yaw, distance, r, L=WB):
    theta = pi_2_pi(distance / r)
    tmp_x = r * sin(theta)
    tmp_y = r - r*cos(theta)
    new_r = sqrt(tmp_x**2 + tmp_y**2)
    theta0 = yaw + atan(tmp_y/tmp_x)
    x += new_r * cos(theta0)
    y += new_r * sin(theta0)
    yaw += theta

    return x, y, yaw


if __name__ == '__main__':
    x, y, yaw = 0., 0., 1.
    plt.axis('equal')
    plot_car(x, y, yaw)
    plt.show()
