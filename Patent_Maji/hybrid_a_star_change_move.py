"""

Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""

import heapq
import scipy.spatial
import numpy as np
import math
import cmath
import matplotlib.pyplot as plt
import xlrd
try:
    from a_star_change_move import dp_planning  # , calc_obstacle_map
    import reeds_shepp_path_planning as rs
    from car_change_move import move, check_car_collision, MAX_STEER, WB, plot_car, new_move, spr_move1, spr_move2, r_move
except:
    raise


XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(0.5)  # [rad]
MOTION_RESOLUTION = 0.1  # [m] path interporate resolution
N_STEER = 20  # number of steer command
H_COST = 1.0
VR = 1.0  # robot radius

SB_COST = 10000.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
H_COST = 1  # Heuristic cost

LEN_SPIRAL = 1

MIN_SEG = 5  # 最小曲线长度的线元段数
show_animation = True


class Node:

    def __init__(self, xind, yind, yawind, direction,
                 xlist, ylist, yawlist, directions,
                 steer=0.0, pind=None, cost=None, catogory=None):
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
        self.catogory = catogory  # -1表示缓和曲线；0表示直线； 1表示曲线


class Path:

    def __init__(self, xlist, ylist, yawlist, directionlist, cost, steer=None):
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directionlist = directionlist
        self.cost = cost
        self.steer = steer


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
        yield [steer, 1]


def get_neighbors(current, config, ox, oy, kdtree, closelist):

    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config,
                              ox, oy, kdtree, closelist)
        if node and verify_index(node, config):
            yield node


def calc_next_node(current, steer, direction, config, ox, oy, kdtree, closelist):

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

    if count >= min_seg and steer*current.steer != 0:
        return None  # 曲线间必须由缓和曲线或直线连接

    if current.catogory is not None and current.steer == 0:
        if closelist[current.pind].steer*steer > 0:
            return None  # 不允许C形曲线

    if steer == 0 and current.steer == 0:  # 判断当前节点的类型，-2为缓和曲线（圆到直）
        cato = 0  # ,-1为缓和曲线（直到圆），0为直线，1为曲线
    elif (steer != 0 and current.steer == 0):
        cato = -1
    elif (steer == 0 and current.steer != 0):
        cato = -2
    elif steer != 0 and current.steer != 0:
        cato = 1
    else:
        cato = None

    x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]
    x_old, y_old, yaw_old = x, y, yaw

    arc_l = LEN_SPIRAL if (cato == -1 or cato == -2) else XY_GRID_RESOLUTION * 1.5
    xlist, ylist, yawlist = [], [], []
    for dist in np.arange(0, arc_l, MOTION_RESOLUTION):
        if cato == -1:
            x, y, yaw = spr_move1(x_old, y_old, yaw_old,
                                  dist+MOTION_RESOLUTION, steer, LEN_SPIRAL)
        elif cato == -2:
            x, y, yaw = spr_move2(
                x_old, y_old, yaw_old, dist+MOTION_RESOLUTION, current.steer, LEN_SPIRAL)
        else:
            x, y, yaw = new_move(
                x, y, yaw, MOTION_RESOLUTION * direction, steer)
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
    addedcost += STEER_COST * abs(steer)

    # steer change penalty
    # + int((abs(current.steer - steer)/(2*MAX_STEER/(N_STEER-1))))**10
    addedcost += STEER_CHANGE_COST * abs(current.steer - steer)

    cost = current.cost + addedcost + arc_l

    node = Node(xind, yind, yawind, d, xlist,
                ylist, yawlist, [d],
                pind=calc_index(current, config),
                cost=cost, steer=steer, catogory=cato)

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
    for i in range(1, n_curvature+1):  # 所有曲率均计算R-S曲线
        max_curvature = math.tan(MAX_STEER*(i/n_curvature)) / WB
        paths = rs.calc_paths(sx, sy, syaw, gx, gy, gyaw,
                              max_curvature, step_size=MOTION_RESOLUTION)
        paths_collect = paths_collect + paths

    paths_selected = []
    for path in paths_collect:
        if path.lengths[0] >= 0 and path.lengths[1] >= 0 and path.lengths[2] >= 0:  # 退化为杜宾曲线
            cur_flag = True
            for i in range(3):
                if path.ctypes[i] != 'S' and path.lengths[i] < MIN_SEG*1.5*XY_GRID_RESOLUTION:  # 最短曲线长度约束
                    cur_flag = False  # 满足最小曲线长度
            if cur_flag:
                paths_selected.append(path)
    paths = paths_selected

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        # TODO: 做到这儿，7月1日凌晨，RS曲线增设缓和曲线，目前写完一段（共三段，e.g.，LSR）的代码
        # TODO：程序需要模块化，并且检验其正确性
        x, y, yaw = sx, sy, syaw
        r = 1/path.curvature
        path.x, path.y, path.yaw, path.cur_type = [], [], [], []
        for i in range(3):
            x, y, yaw, x_out, y_out, yaw_out, len_new_item, type_out = get_x_y_yaw_of_new_rs_part(
                x, y, yaw, path.lengths[i], path.ctypes[i], r)
            path.x += x_out
            path.y += y_out
            path.yaw += yaw_out
            path.cur_type += type_out
            path.lengths[i] = len_new_item
        if check_car_collision(path.x, path.y, path.yaw, ox, oy, kdtree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path
    return best_path


def get_x_y_yaw_of_new_rs_part(sx, sy, syaw, len_item, type_item, r):
    """ 将原rs曲线的不同类型段重构，将圆曲线转为缓和曲线加圆曲线"""
    if type_item == 'R' or type_item == 'L':
        sign_of_r = 1 if type_item == 'L' else -1  # 左转为正，右转为负
        dis = 0
        m1_x, m1_y, m1_yaw = r_move(sx, sy, syaw, len_item, sign_of_r*r)
        sp1x, sp1y, sp1yaw = [], [], []
        sp2x, sp2y, sp2yaw = [], [], []
        cx, cy, cyaw = [], [], []
        rr = get_r(syaw, m1_yaw, r)
        while dis <= LEN_SPIRAL:
            x, y, yaw = spr_move1(sx, sy, syaw, dis, 0,
                                  LEN_SPIRAL, L=WB, radi=sign_of_r*rr)
            sp1x.append(x), sp1y.append(y), sp1yaw.append(yaw)
            x0, y0, yaw0 = spr_move1(
                m1_x, m1_y, m1_yaw+np.pi, dis, 0, LEN_SPIRAL, L=WB, radi=-sign_of_r*rr)
            sp2x.insert(0, x0), sp2y.insert(0, y0), sp2yaw.insert(0, yaw0)
            dis += MOTION_RESOLUTION
        while abs(x0-x)+abs(y0-y) >= 0.1:
            x, y, yaw = r_move(x, y, yaw, MOTION_RESOLUTION, sign_of_r*rr)
            cx.append(x), cy.append(y), cyaw.append(yaw)
        len_new_item = rr*abs(m1_yaw-syaw-LEN_SPIRAL/rr)+2*LEN_SPIRAL
        # plt.plot(sp1x, sp1y, 'xy'), plt.plot(sp2x, sp2y, 'xy'), plt.plot(cx, cy, 'xg')
        x_out = sp1x + cx + sp2x
        y_out = sp1y + cy + sp2y
        yaw_out = sp1yaw + cyaw + sp2yaw
        type_out = [-1 for i in range(len(sp1x))] + [1 for i in range(len(cx))] + [-1 for i in range(len(sp2x))]
    else:
        m1_x, m1_y, m1_yaw = new_move(sx, sy, syaw, len_item, 0)
        dis = MOTION_RESOLUTION
        x_out, y_out, yaw_out = [], [], []
        x, y, yaw = sx, sy, syaw
        while dis <= len_item:
            x, y, yaw = new_move(x, y, yaw, MOTION_RESOLUTION, 0)
            # plt.plot(x,y,'xr')
            x_out.append(x), y_out.append(y), yaw_out.append(yaw)
            dis += MOTION_RESOLUTION
        len_new_item = len_item
        type_out = [0 for i in range(len(x_out))]
    return m1_x, m1_y, m1_yaw, x_out, y_out, yaw_out, len_new_item, type_out


def get_r(syaw, gyaw, r):  # RS曲线的圆弧段修正为缓和曲线+圆弧，此函数用于求新的曲线半径,缓和曲线长度已知
    alpha = abs(gyaw - syaw)
    ta2 = np.tan(alpha/2)
    TT = r*ta2
    ls = LEN_SPIRAL
    # ls*ta2*(1+(1/24)*kk**2-(1/2688)*kk**4)+ls*(0.5*kk-(1/240)*kk**3) - TT*kk == 0 # 其中ls/R=k
    a = -(1/2688)*ls*ta2
    b = -(1/240)*ls
    c = (1/24)*ls*ta2
    d = 0.5*ls-TT
    e = ls*ta2
    P = (c**2+12*a*e-3*b*d)/9
    Q = (27*a*d**2+2*c**3+27*b**2*e-72*a*c*e-9*b*c*d)/54
    D = cmath.sqrt(Q**2-P**3)
    u = (Q+D)**(1/3) if abs(Q+D) >= abs(Q-D) else (Q-D)**(1/3)
    v = 0 if u == 0 else P/u
    w = complex(-0.5, 3**0.5/2)
    m = []
    M = []
    flag = 0
    for i in range(3):
        x = cmath.sqrt(b**2-8*a*c/3+4*a*(w**i*u+w**(3-i)*v))
        m.append(x)
        M.append(abs(x))
        if m == 0:
            flag = flag+1
    if flag == 3:
        mm = 0
        S = b**2-8*a*c/3
        T = 0
    else:
        t = M.index(max(M))
        mm = m[t]
        S = 2*b**2-16*a*c/3-4*a*(w**t*u+w**(3-t)*v)
        T = (8*a*b*c-16*a**2*d-2*b**3)/mm
    x1 = (-b-mm+cmath.sqrt(S-T))/(4*a)
    R = ls/abs(x1)
    x2 = (-b-mm-cmath.sqrt(S-T))/(4*a)
    if abs(R-r) > abs(ls/abs(x2)-r):
        R = ls/abs(x2)
    x3 = (-b+mm+cmath.sqrt(S+T))/(4*a)
    if abs(R-r) > abs(ls/abs(x3)-r):
        R = ls/abs(x3)
    x4 = (-b+mm-cmath.sqrt(S+T))/(4*a)
    if abs(R-r) > abs(ls/abs(x4)-r):
        R = ls/abs(x4)

    return R


def update_node_with_analystic_expantion(current, goal,
                                         c, ox, oy, kdtree):
    apath = analytic_expantion(current, goal, c, ox, oy, kdtree)

    if apath:
        # plt.plot(apath.x, apath.y)
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw = apath.yaw[1:]

        for i in range(len(apath.cur_type)):
            if apath.cur_type[i] == -1:
                plt.plot(apath.x[i],apath.y[i],'xy')
            elif apath.cur_type[i] == 0:
                plt.plot(apath.x[i],apath.y[i],'xr')
            else:
                plt.plot(apath.x[i],apath.y[i],'xg')

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
    if current.steer != 0:
        return False  # RS只能接直线，连缓和曲线都不能接
    if count > 0 and count < min_seg:
        return False
    else:
        return True


def hybrid_a_star_planning(start, goal, ox, oy, xyreso, yawreso):
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

    _, _, h_dp = dp_planning(nstart.xlist[-1], nstart.ylist[-1],
                             ngoal.xlist[-1], ngoal.ylist[-1], ox, oy, xyreso, VR)

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
            # plt.plot(current.xlist[-1], current.ylist[-1], "xc")
            if len(closedList.keys()) % 4000 == 0:
                plt.plot(current.xlist[-1], current.ylist[-1], "xc")
                plt.pause(0.001)

        isupdated = None
        # abs(current.xlist[-1]-ngoal.xlist[-1])+abs(current.ylist[-1]-ngoal.ylist[-1]) < 10:
        if check_rs_permition(current, closedList, ngoal) and \
                abs(current.xlist[-1]-ngoal.xlist[-1])+abs(current.ylist[-1]-ngoal.ylist[-1]) < 40:
            isupdated, fpath = update_node_with_analystic_expantion(
                current, ngoal, config, ox, oy, obkdtree)

        if isupdated:
            break

        for neighbor in get_neighbors(current, config, ox, oy, obkdtree, closedList):
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

        if n.catogory == 0:
            plt.plot(n.xlist, n.ylist, ".r")
        elif n.catogory == 1:
            plt.plot(n.xlist, n.ylist, ".g")
        elif n.catogory == -1 or n.catogory == -2:
            plt.plot(n.xlist, n.ylist, ".b")

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
    ind = abs(node.yawind) * c.xw * c.yw + \
        (node.yind - c.miny) * c.xw + (node.xind - c.minx)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind


def main():
    print("Start Hybrid A* planning")

    ox, oy = [], []

    data_filename = 'maji.xls'
    work_Book = xlrd.open_workbook(data_filename)
    sheet = work_Book.sheet_by_name('Sheet1')
    for i in range(0, sheet.nrows):
        cells = sheet.row_values(i)
        # 保留两位小数
        ox.append(int(cells[0]))
        oy.append(int(cells[1]))

    # Set Initial parameters
    # start = [53.21, 74.08, np.deg2rad(180.0)]
    # goal = [88.26, 130.29, np.deg2rad(180.0)]
    # start = [88.26, 130.29, np.deg2rad(0.0)]
    # goal = [53.21, 74.08, np.deg2rad(0.0)]
    # goal = [119.26, 40.29, np.deg2rad(90.0)]
    start = [40.0, 20.0, np.deg2rad(90.0)]
    goal = [20.0, 40.0, np.deg2rad(180.0)]

    plt.plot(ox, oy, ".k")
    rs.plot_arrow(start[0], start[1], start[2], fc='g')
    rs.plot_arrow(goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")

    path = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    # x = path.xlist
    # y = path.ylist
    # yaw = path.yawlist

    # for ix, iy, iyaw in zip(x, y, yaw):
    #     # plt.cla()
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(x, y, ".r", label="Hybrid A* path")
    #     plt.grid(True)
    #     plt.axis("equal")
    #     plot_car(ix, iy, iyaw)
    #     plt.pause(0.0001)
    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print(__file__ + " done!!")


if __name__ == '__main__':
    main()
