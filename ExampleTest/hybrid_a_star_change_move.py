"""

Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""

import heapq
import scipy.spatial
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import xlrd
sys.path.append("../ReedsSheppPath/")
try:
    from a_star_change_move import dp_planning  # , calc_obstacle_map
    import reeds_shepp_path_planning as rs
    from car_change_move import move, check_car_collision, MAX_STEER, WB, plot_car
except:
    raise


XY_GRID_RESOLUTION = 6.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(3.0)  # [rad]
MOTION_RESOLUTION = 0.3  # [m] path interporate resolution
N_STEER = 20  # number of steer command
VR = 7.0  # robot radius

SB_COST = 10000.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
H_COST = XY_GRID_RESOLUTION*10  # Heuristic cost

MAX_STEER = 0.022
MIN_SEG = 6  # 最小曲线长度的线元段数
show_animation = True



class Node:

    def __init__(self, xind, yind, yawind, direction,
                 xlist, ylist, yawlist, directions,
                 steer=0.0, pind=None, cost=None):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directions = directions
        self.steer = steer
        self.pind = pind
        self.cost = cost


class Path:

    def __init__(self, xlist, ylist, yawlist, directionlist, cost):
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directionlist = directionlist
        self.cost = cost


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


class Config:

    def __init__(self, ox, oy, xyreso, yawreso):
        min_x_m = min(ox)
        min_y_m = min(oy)
        max_x_m = max(ox)
        max_y_m = max(oy)

        ox.append(min_x_m)
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.minx = round(min_x_m / xyreso)
        self.miny = round(min_y_m / xyreso)
        self.maxx = round(max_x_m / xyreso)
        self.maxy = round(max_y_m / xyreso)

        self.xw = round(self.maxx - self.minx)
        self.yw = round(self.maxy - self.miny)

        self.minyaw = round(- math.pi / yawreso) - 1
        self.maxyaw = round(math.pi / yawreso)
        self.yaww = round(self.maxyaw - self.minyaw)


def calc_motion_inputs():

    for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER, N_STEER), [0.0])):
        for d in [1]:
            yield [steer, d]


def get_neighbors(current, config, ox, oy, kdtree, closelist, ermap):

    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kdtree, closelist, ermap)
        if node and verify_index(node, config):
            yield node


def calc_next_node(current, steer, direction, config, ox, oy, kdtree, closelist, ermap):

    min_seg = MIN_SEG
    count, tmp = 0, current  # count 用来判断当前曲线是否满足约束，能否开始RS拟合
    while (tmp.pind != None and tmp.steer != 0):
        tst = closelist[tmp.pind]
        count += 1
        if tmp.steer != tst.steer:
            break
        tmp = tst  # 检测相同曲率的圆曲线段数
    if (count > 0 and count < min_seg) and steer != current.steer:
        return None  # 不足曲线长度约束按原半径继续探索
    if count >= min_seg and steer*current.steer > 0:
        return None  # 满足最小曲线半径后如果变化，只能反向，防止C形曲线
        
    x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]

    arc_l = XY_GRID_RESOLUTION * 1.5
    xlist, ylist, yawlist = [], [], []
    for dist in np.arange(0, arc_l, MOTION_RESOLUTION):
        x, y, yaw = move(x, y, yaw, MOTION_RESOLUTION * direction, steer)
        xlist.append(x)
        ylist.append(y)
        yawlist.append(yaw)

    if not check_car_collision(xlist, ylist, yawlist, ox, oy, kdtree):
        return None

    d = direction == 1
    xind = round(x / XY_GRID_RESOLUTION)
    yind = round(y / XY_GRID_RESOLUTION)
    yawind = round(yaw / YAW_GRID_RESOLUTION)

    addedcost = 0.0

    if d != current.direction:
        addedcost += SB_COST

    # steer penalty
    addedcost += STEER_COST * abs(steer)*0.1  # 基本无作用，用来在角度分辨率中尽量选择直线

    # steer change penalty
    # + int((abs(current.steer - steer)/(2*MAX_STEER/(N_STEER-1))))**10
    # addedcost += STEER_CHANGE_COST * abs(current.steer - steer)

    e_sign = 0 if ermap[xind - config.minx][yind - config.miny] else 1
    cost = current.cost + addedcost + arc_l*e_sign

    node = Node(xind, yind, yawind, d, xlist,
                ylist, yawlist, [d],
                pind=calc_index(current, config),
                cost=cost, steer=steer)

    return node


def is_same_grid(n1, n2):
    if n1.xind == n2.xind and n1.yind == n2.yind and n1.yawind == n2.yawind:
        return True
    return False


def analytic_expantion(current, goal, c, ox, oy, kdtree):

    sx = current.xlist[-1]
    sy = current.ylist[-1]
    syaw = current.yawlist[-1]

    gx = goal.xlist[-1]
    gy = goal.ylist[-1]
    gyaw = goal.yawlist[-1]

    n_curvature, paths_collect = round(N_STEER/2), []
    for i in range(1,n_curvature+1):  #  所有曲率均计算R-S曲线
        max_curvature = math.tan(MAX_STEER*(i/n_curvature)) / WB
        paths = rs.calc_paths(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size=MOTION_RESOLUTION)
        paths_collect = paths_collect + paths

    paths_selected = []
    for path in paths_collect:
        if path.lengths[0] >= 0 and path.lengths[1] >= 0 and path.lengths[2] >= 0:  # 退化为杜宾曲线
            cur_flag = True
            for i in range(3):
                if path.ctypes[i] != 'S' and path.lengths[i] < MIN_SEG*1.5*XY_GRID_RESOLUTION:  # 最短曲线长度约束
                    cur_flag = False
            if cur_flag:
                paths_selected.append(path)
    paths = paths_selected

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        if check_car_collision(path.x, path.y, path.yaw, ox, oy, kdtree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path

    return best_path


def update_node_with_analystic_expantion(current, goal,
                                         c, ox, oy, kdtree):
    apath = analytic_expantion(current, goal, c, ox, oy, kdtree)

    if apath:
        plt.plot(apath.x, apath.y)
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw = apath.yaw[1:]

        fcost = current.cost + calc_rs_path_cost(apath)
        fpind = calc_index(current, c)

        fd = []
        for d in apath.directions[1:]:
            fd.append(d >= 0)

        fsteer = 0.0
        fpath = Node(current.xind, current.yind, current.yawind,
                     current.direction, fx, fy, fyaw, fd,
                     cost=fcost, pind=fpind, steer=fsteer)
        return True, fpath

    return False, None


def calc_rs_path_cost(rspath):

    cost = 0.0
    for l in rspath.lengths:
        if l >= 0:  # forward
            cost += l
        else:  # back
            cost += abs(l) * BACK_COST

    # swich back penalty
    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S":  # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = [0.0] * nctypes
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER

    for i in range(len(rspath.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    return cost

def check_rs_permition(current, closedList, ngoal):
    min_seg = MIN_SEG
    count, tmp = 0, current  # count 用来判断当前曲线是否满足约束，能否开始RS拟合
    while (tmp.pind != None and tmp.steer != 0):
        tst = closedList[tmp.pind]
        count += 1
        if tmp.steer != tst.steer:
            break
        tmp = tst  # 检测相同曲率的圆曲线段数
    if count > 0 and count < min_seg:
        return False
    else:
        return True

def hybrid_a_star_planning(start, goal, ox, oy, ex, ey, xyreso, yawreso):
    """
    start
    goal
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """
    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    tox, toy = ox[:], oy[:]

    obkdtree = KDTree(np.vstack((tox, toy)).T)

    config = Config(tox, toy, xyreso, yawreso)

    nstart = Node(round(start[0] / xyreso), round(start[1] / xyreso), round(start[2] / yawreso),
                  True, [start[0]], [start[1]], [start[2]], [True], cost=0)
    ngoal = Node(round(goal[0] / xyreso), round(goal[1] / xyreso), round(goal[2] / yawreso),
                 True, [goal[0]], [goal[1]], [goal[2]], [True])

    openList, closedList = {}, {}

    _, _, h_dp, ermap = dp_planning(nstart.xlist[-1], nstart.ylist[-1],
                             ngoal.xlist[-1], ngoal.ylist[-1], ox, oy, xyreso, VR, ex, ey)

    pq = []
    openList[calc_index(nstart, config)] = nstart
    heapq.heappush(pq, (calc_cost(nstart, h_dp, ngoal, config),
                        calc_index(nstart, config)))

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else:
            continue

        if show_animation:  # pragma: no cover
            plt.plot(current.xlist[-1], current.ylist[-1], "xc")
            if len(closedList.keys()) % 2 == 0:
                plt.pause(0.001)

        isupdated = None
        # abs(current.xlist[-1]-ngoal.xlist[-1])+abs(current.ylist[-1]-ngoal.ylist[-1]) < 10:
        if check_rs_permition(current, closedList, ngoal) and \
            abs(current.xlist[-1]-ngoal.xlist[-1])+abs(current.ylist[-1]-ngoal.ylist[-1]) < 170:
            isupdated, fpath = update_node_with_analystic_expantion(
                current, ngoal, config, ox, oy, obkdtree)

        if isupdated:
            break

        for neighbor in get_neighbors(current, config, ox, oy, obkdtree, closedList, ermap):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList:
                continue
            if neighbor not in openList \
                    or openList[neighbor_index].cost > neighbor.cost:
                heapq.heappush(
                    pq, (calc_cost(neighbor, h_dp, ngoal, config),
                         neighbor_index))
                openList[neighbor_index] = neighbor

    path = get_final_path(closedList, fpath, nstart, config)
    return path


def calc_cost(n, h_dp, goal, c):
    ind = (n.yind - c.miny) * c.xw + (n.xind - c.minx)
    if ind not in h_dp:
        return n.cost + 999999999  # collision cost
    return n.cost + H_COST * h_dp[ind].cost


def get_final_path(closed, ngoal, nstart, config):
    rx, ry, ryaw = list(reversed(ngoal.xlist)), list(
        reversed(ngoal.ylist)), list(reversed(ngoal.yawlist))
    direction = list(reversed(ngoal.directions))
    nid = ngoal.pind
    finalcost = ngoal.cost

    while nid:
        n = closed[nid]
        rx.extend(list(reversed(n.xlist)))
        ry.extend(list(reversed(n.ylist)))
        ryaw.extend(list(reversed(n.yawlist)))
        direction.extend(list(reversed(n.directions)))

        nid = n.pind

    rx = list(reversed(rx))
    ry = list(reversed(ry))
    ryaw = list(reversed(ryaw))
    direction = list(reversed(direction))

    # adjust first direction
    direction[0] = direction[1]

    path = Path(rx, ry, ryaw, direction, finalcost)

    return path


def verify_index(node, c):
    xind, yind = node.xind, node.yind
    if xind >= c.minx and xind <= c.maxx and yind >= c.miny \
            and yind <= c.maxy:
        return True

    return False


def calc_index(node, c):
    ind = (node.yawind - c.minyaw) * c.xw * c.yw + \
        (node.yind - c.miny) * c.xw + (node.xind - c.minx)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind


def main():
    print("Start Hybrid A* planning")

    ox, oy = [], []
    data_filename = './ExampleTest/line_points.xls'
    work_Book = xlrd.open_workbook(data_filename)
    sheet = work_Book.sheet_by_name('sheet1')
    ox = []
    oy = []
    for i in range(0, sheet.nrows):
        cells = sheet.row_values(i)
        # 保留两位小数
        ox.append(round(float(cells[0]), 2))
        oy.append(round(float(cells[1]), 2))
    
    ex, ey = [], []
    data_filename1 = './ExampleTest/line_exist.xls'
    work_Book1 = xlrd.open_workbook(data_filename1)
    sheet1 = work_Book1.sheet_by_name('sheet1')
    for i in range(0, sheet1.nrows):
        cells = sheet1.row_values(i)
        # 保留两位小数
        ex.append(round(float(cells[0]), 2))
        ey.append(round(float(cells[1]), 2))

    # Set Initial parameters
    start = [200.0, 1370.0, np.deg2rad(0.0)]
    # start = [1025.0, 550.0, np.deg2rad(-45.0)]
    goal = [1150.0, 200.0, np.deg2rad(-70.0)]

    plt.plot(ox, oy, ".k")
    rs.plot_arrow(start[0], start[1], start[2], fc='g')
    rs.plot_arrow(goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")

    path = hybrid_a_star_planning(
        start, goal, ox, oy, ex, ey, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    x = path.xlist
    y = path.ylist
    yaw = path.yawlist

    for ix, iy, iyaw in zip(x, y, yaw):
        plt.cla()
        plt.plot(ox, oy, ".k")
        plt.plot(x, y, "-r", label="Hybrid A* path")
        plt.grid(True)
        plt.axis("equal")
        plot_car(ix, iy, iyaw)
        plt.pause(0.0001)

    print(__file__ + " done!!")


if __name__ == '__main__':
    main()
