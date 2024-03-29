"""

A* grid based planning

author: Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
import xlrd

show_animation = False


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def dp_planning(sx, sy, gx, gy, ox, oy, reso, rr, ex, ey):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """
    # global frame
    # global artists
    nstart = Node(round(sx), round(sy), 0.0, -1)
    ngoal = Node(round(gx), round(gy), 0.0, -1)

    obmap, ermap, minx, miny, maxx, maxy, xw, yw, costmap = calc_obstacle_map(
        ox, oy, reso, rr, ex, ey)
    
    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(ngoal, xw, minx, miny)] = ngoal
    pq = []
    pq.append((0, calc_index(ngoal, xw, minx, miny)))

    while 1:
        if not pq:
            break
        cost, c_id = heapq.heappop(pq)
        if c_id in openset:
            current = openset[c_id]
            closedset[c_id] = current

            costmap[current.x][current.y] = current.cost

            openset.pop(c_id)
        else:
            continue

        # show graph
        if show_animation:  # pragma: no cover
            # plt.plot(current.x * reso, current.y * reso, "xc")
            # if len(closedset.keys()) % 10 == 0:
            #     # artists.append(frame)
            #     plt.pause(0.001)
            pass

        # Remove the item from the open set

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            new_x = current.x + motion[i][0]
            new_y = current.y + motion[i][1]
            e_sign = 0 if ermap[new_x-minx][new_y-miny] else 1
            new_cost = current.cost + motion[i][2] * e_sign
            node = Node(new_x,
                        new_y,
                        new_cost, c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
                heapq.heappush(
                    pq, (node.cost, calc_index(node, xw, minx, miny)))
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node
                    heapq.heappush(
                        pq, (node.cost, calc_index(node, xw, minx, miny)))

    rx, ry = calc_final_path(closedset[calc_index(
        nstart, xw, minx, miny)], closedset, reso)
    np.savez('maps', obmap=obmap, ermap=ermap, costmap=costmap)
    return rx, ry, closedset, ermap


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x-minx][node.y-miny]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr, ex, ey):

    minx = 0
    miny = 0
    maxx = max(ox)+1
    maxy = max(oy)+1

    xwidth = maxx
    ywidth = maxy

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    ermap = [[False for i in range(ywidth)] for i in range(xwidth)]
    costmap = [[-1.0 for i in range(ywidth)] for i in range(xwidth)]
    for i in range(len(ox)):
        obmap[ox[i]][oy[i]] = True
    
    for i in range(len(ex)):
        ermap[ex[i]][ey[i]] = True

    return obmap, ermap, minx, miny, maxx, maxy, xwidth, ywidth, costmap


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    global frame
    global artists

    sx = 11.0  # [m]
    sy = 50.0  # [m]
    gx = 597.0  # [m]
    gy = 157.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]
    ox, oy = [], []
    data_filename = './ChengduCase/Border.xls'
    work_Book = xlrd.open_workbook(data_filename)
    sheet = work_Book.sheet_by_name('Border')
    for i in range(0, sheet.nrows):
        cells = sheet.row_values(i)
        # 保留两位小数
        ox.append(int(cells[0]))
        oy.append(int(cells[1]))

    ex, ey = [], []
    data_filename1 = './ChengduCase/ERoad.xls'
    work_Book1 = xlrd.open_workbook(data_filename1)
    sheet1 = work_Book1.sheet_by_name('ERoad')
    for i in range(0, sheet1.nrows):
        cells = sheet1.row_values(i)
        # 保留两位小数
        ex.append(int(cells[0]))
        ey.append(int(cells[1]))

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(ex, ey, ".y")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry, _, _ = dp_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_size, ex, ey)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    show_animation = True
    # fig = plt.figure()
    # global frame
    # global artists
    # frame, artists = [], []
    main()
    # [artists.append(frame) for i in range (10)]
    # animation = ani.ArtistAnimation(fig = fig, artists=artists)
    # animation.save('a_star.gif',fps = 30)
