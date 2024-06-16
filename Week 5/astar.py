import pandas as pd
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from collections import deque
import heapq
import time


class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist  # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a);
    t2 = (-b - np.sqrt(discriminant)) / (2 * a);

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True


def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(stepSize, length)

    newvex = (nearvex[0] + dirn[0], nearvex[1] + dirn[1])
    return newvex


def window(startpos, endpos):
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height


def isInWindow(pos, winx, winy, width, height):
    if winx < pos[0] < winx + width and \
            winy < pos[1] < winy + height:
        return True
    else:
        return False


class Graph:
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        return posx, posy


def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            # print('success')
            # break
    return G


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist

            G.success = True
            # print('success')
            # break
    return G


def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def Astar(G):
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build A* priority queue
    pq = [(0, srcIdx, [])]
    seen = {srcIdx: 0}

    while pq:
        (dist, curNode, path) = heapq.heappop(pq)

        if curNode == dstIdx:
            return [G.vertices[i] for i in path]

        for neighbor, cost in G.neighbors[curNode]:
            old_cost = seen.get(neighbor, None)
            new_cost = dist + cost

            if old_cost is None or new_cost < old_cost:
                seen[neighbor] = new_cost
                heapq.heappush(pq, (new_cost + heuristic(G.vertices[neighbor], G.endpos), neighbor, path + [neighbor]))

    return []


def path_length(path):
    return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))


def plot(G, obstacles, radius, path=None, path_length=None, time_taken=None):
    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    if path_length is not None and time_taken is not None:
        plt.text(0.05, 1.19, 'Path Length: {:.2f}units\nTime Taken: {:.2f}sec\nMethod: Astar'.format(path_length, time_taken),
                 verticalalignment='top', horizontalalignment='left',
                 transform=ax.transAxes, color='blue', fontsize=15)

    ax.autoscale()
    ax.margins(0.1)
    plt.subplots_adjust(top=0.85)
    plt.show()


def loadData(filename):
    df = pd.read_csv("highway_map.csv")

    # Extract coordinates
    coords = df[['x', 'y']].values.tolist()

    # Set start and end positions
    startpos = tuple(coords[0])
    endpos = tuple(coords[1])

    # Set obstacles
    obstacles = [tuple(coord) for coord in coords[2:]]

    return startpos, endpos, obstacles


def plot_given_path(startpos, endpos, path, obstacles, radius):
    px = [x for x, y in path]
    py = [y for x, y in path]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(startpos[0], startpos[1], c='black')
    ax.scatter(endpos[0], endpos[1], c='black')

    paths = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
    lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
    ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    start_time = time.time()
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)  # change here
    if G.success:
        path = Astar(G)  # change here
        end_time = time.time()
        path_length_val = path_length(path)
        time_taken = end_time - start_time
        return G, path, path_length_val, time_taken


if __name__ == '__main__':
    filename = "highway_map.csv"
    startpos, endpos, obstacles = loadData(filename)

    n_iter = 200
    radius = 0.5
    stepSize = 0.7

    G, path, path_length_val, time_taken = pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize)  # change here

    if path is not None:
        print(path)
        plot(G, obstacles, radius, path, path_length_val, time_taken)  # G is now defined
    else:
        print("No path found.")

