# -*- coding: utf-8 -*-
from itertools import permutations
from tkinter import *
import numpy as np
from queue import Queue
from queue import PriorityQueue
import sys
menu = Tk()
def onClose():
    sys.exit()

menu.protocol("WM_DELETE_WINDOW", onClose)

label = Label(menu, text="Vui lòng chọn thuật toán thực hiện", relief=RAISED)

option = 0

def bfs_callback():
    global option
    option = 1
    menu.destroy()


def dfs_callback():
    global option
    option = 2
    menu.destroy()


def dij_callback():
    global option
    option = 3
    menu.destroy()


def a_callback():
    global option
    option = 4
    menu.destroy()

def pfw_callback():
    global option
    option = 5
    menu.destroy()


btn_bfs = Button(menu, text="BFS", command=bfs_callback)
btn_dfs = Button(menu, text="DFS", command=dfs_callback)
btn_dijkstra = Button(menu, text="Dijkstra", command=dij_callback)
btn_aStar = Button(menu, text="A Star", command=a_callback)
btn_pfw = Button(menu, text="Path finding with visited node (BFS)", command=pfw_callback)
label.pack()

btn_bfs.pack()
btn_dfs.pack()
btn_dijkstra.pack()
btn_aStar.pack()
btn_pfw.pack()

mainloop()

# --------------------------menu--------------------


np.set_printoptions(threshold=sys.maxsize)

master = Tk()
master.protocol("WM_DELETE_WINDOW", onClose)

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Polygon:
    def __init__(self, num, array):
        self.num = num
        self.arr = array


horizontal_axis = 0  # khởi tạo trục hoành
vertical_axis = 0  # khởi tạo trục tung

s = Point(0, 0)  # khởi tạo điểm bắt đầu
g = Point(0, 0)  # khởi tạo điểm đích
num_pol = 0  # số đa giác
array_polygon = []  # mảng các đa giác
array_pickup_point = []  # mảng các điểm đón


# ---------------------------------------------------------------------------------------------------

# đọc file

def input(file_name):
    horizontal_axis = 0
    vertical_axis = 0
    s = Point(0, 0)
    g = Point(0, 0)
    array_pickup_point = []
    num_pol = 0
    array_polygon = []
    i = 0
    with open(file_name) as file:
        line = file.readlines()
        for k in range(0, len(line)):
            if k == 0:
                data = line[k].strip().split(',')
                horizontal_axis = int(data[0])
                vertical_axis = int(data[1])
            if k == 1:
                data = line[k].strip().split(',')
                s.x = int(data[0])
                s.y = int(data[1])
                g.x = int(data[2])
                g.y = int(data[3])
                if (len(data) > 4):
                    m = 2
                    n = 0
                    while m < int(len(data) / 2):
                        array_pickup_point.append(Point(0, 0))
                        array_pickup_point[n].x = int(data[2 * m])
                        array_pickup_point[n].y = int(data[2 * m + 1])
                        n += 1
                        m += 1

            if k == 2:
                data = line[k].strip(' ')
                num_pol = int(data[0])
            elif k > 2:
                array_point = []
                array_polygon.append(Polygon(0, array_point))
                data = line[k].strip().split(',')
                for count in range(0, int(len(data) / 2)):
                    array_point.append(Point(0, 0))
                    array_point[count].x = (int(data[2 * count]))
                    array_point[count].y = (int(data[2 * count + 1]))
                array_polygon[i].num = int(len(array_point))
                for j in range(0, array_polygon[i].num):
                    array_polygon[i].arr.append(Point(0, 0))
                    array_polygon[i].arr[j] = array_point[j]
                i += 1
    return horizontal_axis, vertical_axis, s, g, array_pickup_point, num_pol, array_polygon


horizontal_axis, vertical_axis, s, g, array_pickup_point, num_pol, array_polygon = input('input.txt')

p = 20  # p là độ rộng của mỗi ô vuông
size_font = 20  # size chữ

if (horizontal_axis > 50) | (vertical_axis > 50):
    p = 15
    size_font = 10
elif (horizontal_axis < 50) & (vertical_axis < 50):
    p = 20
    size_font = 12

width = (horizontal_axis + 2) * p
height = (vertical_axis + 2) * p
w = Canvas(master, width=(horizontal_axis + 2) * p, height=(vertical_axis + 2) * p)
x = y = 0
column = row = 0
while row <= height / p:
    x = 0
    column = 0
    while column <= width / p:
        w.create_rectangle(x, y, x + p, y + p, fill='white smoke')
        x += p
        column += 1
    y += p
    row += 1

x = y = 0
column = row = 0
while column <= width / p:
    w.create_rectangle(x, 0, x + p, p, fill='gray50')
    x += p
    column += 1

x = y = 0
column = row = 0
while row <= height / p:
    w.create_rectangle(0, y, p, y + p, fill='gray50')
    y += p
    row += 1

column = row = 0
x = p + p / 2
y = p / 2
k = 0
while column <= width / p:
    w.create_text(x, y, font=('Impact', size_font), text=str(k))
    x += p
    k = int(k)
    k += 1
    column += 1

column = row = 0
x = p / 2
y = p + p / 2
k = 0
while row <= height / p:
    w.create_text(x, y, font=('Impact', size_font), text=str(k))
    y += p
    k = int(k)
    k += 1
    row += 1

w.pack()

array = np.zeros((vertical_axis + 1, horizontal_axis + 1))


def isInPolygon(polygon, x, y):
    j = polygon.num - 1
    c = 0
    for i in range(polygon.num):
        if (polygon.arr[i].y == y) & (polygon.arr[i].x == x):
            return 1
        if polygon.arr[i].y == polygon.arr[j].y:
            if (polygon.arr[i].y == y) & ((x - polygon.arr[i].x) * (x - polygon.arr[j].x) <= 0):
                return 1
        if (polygon.arr[i].y > y) != (polygon.arr[j].y > y):
            dx = (polygon.arr[j].x - polygon.arr[i].x) * (y - polygon.arr[i].y) / (
                    polygon.arr[j].y - polygon.arr[i].y) + polygon.arr[i].x
            if polygon.arr[j].x == polygon.arr[i].x:
                if polygon.arr[j].x == x:
                    return 1
            if (x - dx <= 0.5) & (x - dx >= -0.5):
                return 1
            if x < dx:
                c = c + 1
        j = i
    return c


for i in range(0, num_pol):
    for m in range(0, horizontal_axis + 1):
        for n in range(0, vertical_axis + 1):
            if isInPolygon(array_polygon[i], m, n) == 1:
                array[n][m] = 1
                w.create_rectangle((m + 1) * p, (n + 1) * p, (m + 1) * p + p, (n + 1) * p + p, fill='OrangeRed4')
            for k in range(array_polygon[i].num):
                if (array_polygon[i].arr[k].x == m) & (array_polygon[i].arr[k].y == n):
                    w.create_rectangle((m + 1) * p, (n + 1) * p, (m + 1) * p + p, (n + 1) * p + p, fill='OliveDrab4')


def getNeighborNode(point):
    a = []

    if 0 <= point.x - 1 <= horizontal_axis and 0 <= point.y <= vertical_axis:
        a.append(Point(point.x - 1, point.y))

    if 0 <= point.x + 1 <= horizontal_axis and 0 <= point.y <= vertical_axis:
        a.append(Point(point.x + 1, point.y))

    if 0 <= point.x <= horizontal_axis and 0 <= point.y - 1 <= vertical_axis:
        a.append(Point(point.x, point.y - 1))

    if 0 <= point.x <= horizontal_axis and 0 <= point.y + 1 <= vertical_axis:
        a.append(Point(point.x, point.y + 1))

    return a


def BFS():
    array[s.y][s.x] = 1
    path = [[Point(-1, -1) for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    q = Queue()
    q.put(s)
    while not q.empty():
        u = q.get()
        neighbor = getNeighborNode(u)

        for v in neighbor:
            if array[v.y][v.x] == 0:
                array[v.y][v.x] = 1
                q.put(v)
                path[v.x][v.y] = u

                if v.x == g.x and v.y == g.y:
                    return path
    return path


def DFS():
    array[s.y][s.x] = 1
    path = [[Point(-1, -1) for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    stack = []
    stack.append(s)
    while not len(stack) == 0:
        u = stack.pop()
        neighbor = getNeighborNode(u)
        for v in neighbor:
            if array[v.y][v.x] == 0:
                array[v.y][v.x] = 1
                stack.append(v)
                path[v.x][v.y] = u
                if v.x == g.x and v.y == g.y:
                    return path
    return path


class dijkstraNode():
    def __init__(self, point, w):
        self.point = point
        self.w = w

    def __lt__(self, other):
        return self.w <= other.w


def dijkstra():
    array[s.y][s.x] = 1
    inf = int(1000)
    dist = [[inf for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    path = [[Point(-1, -1) for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    pq = PriorityQueue()
    pq.put(dijkstraNode(s, 0))
    dist[s.x][s.y] = 0
    while not pq.empty():
        top = pq.get()
        point = top.point
        w = top.w
        for neighBor in getNeighborNode(point):
            if w + 1 < dist[neighBor.x][neighBor.y] and array[neighBor.y][neighBor.x] == 0:
                dist[neighBor.x][neighBor.y] = w + 1
                pq.put(dijkstraNode(neighBor, dist[neighBor.x][neighBor.y]))
                path[neighBor.x][neighBor.y] = point
    return path


class aStarNode:
    def __init__(self, point, h):
        inf = 1000
        self.point = point
        self.h = h
        self.f = 10000
        self.g = 10000

    def __lt__(self, other):
        return self.f <= other.f


def aStar():
    array[s.y][s.x] = 1

    # tính heuristic bằng BFS
    heuristic = [[0 for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    bfsVisted = [[0 for x in range(horizontal_axis + 1)] for y in range(vertical_axis + 1)]
    for i in range(0, len(array)):
        for j in range(0, len(array[i])):
            bfsVisted[i][j] = array[i][j]
    q = Queue()
    q.put(g)
    bfsVisted[s.y][s.x] = 0
    bfsVisted[g.y][g.x] = 1

    while not q.empty():
        u = q.get()
        neighbor = getNeighborNode(u)
        for v in neighbor:
            if bfsVisted[v.y][v.x] == 0:
                bfsVisted[v.y][v.x] = 1
                q.put(v)
                heuristic[v.x][v.y] = heuristic[u.x][u.y] + 1

    path = [[Point(-1, -1) for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    s_node = aStarNode(s, heuristic[s.x][s.y])
    s_node.g = 0
    s_node.f = s_node.g

    pq = PriorityQueue()

    pq.put(s_node)

    while not pq.empty():
        current_node = pq.get()
        if current_node.point.x == g.x and current_node.point.y == g.y:
            return path
        array[current_node.point.y][current_node.point.x] = 1
        for neightPoint in getNeighborNode(current_node.point):
            neighBorNode = aStarNode(neightPoint, heuristic[neightPoint.x][neightPoint.y])
            if array[neighBorNode.point.y][neighBorNode.point.x] != 0:
                continue
            temp = True
            for node in pq.queue:
                if node.point.x == neighBorNode.point.x and node.point.y == neighBorNode.point.y:
                    neighBorNode = node
                    new_g = current_node.g + 1
                    new_f = new_g + neighBorNode.h
                    if new_f < neighBorNode.f:
                        neighBorNode.g = new_g
                        neighBorNode.f = new_f
                        path[neighBorNode.point.x][neighBorNode.point.y] = current_node.point
                        temp == False
                        break
            if temp:
                neighBorNode.g = current_node.g + 1
                neighBorNode.f = neighBorNode.g + neighBorNode.h
                path[neighBorNode.point.x][neighBorNode.point.y] = current_node.point
                pq.put(neighBorNode)

    return path


def BFS_2Point(A, B):
    array[A.y][A.x] = 1

    bfsVisted = [[0 for x in range(horizontal_axis + 1)] for y in range(vertical_axis + 1)]
    for i in range(0, len(array)):
        for j in range(0, len(array[i])):
            bfsVisted[i][j] = array[i][j]

    path = [[Point(-1, -1) for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    q = Queue()
    q.put(A)
    while not q.empty():
        u = q.get()
        neighbor = getNeighborNode(u)

        for v in neighbor:
            if bfsVisted[v.y][v.x] == 0:
                bfsVisted[v.y][v.x] = 1
                q.put(v)
                path[v.x][v.y] = u

                if v.x == B.x and v.y == B.y:
                    return path
    return path


def pathFindingWithVistedPoint():
    # tính heuristic bằng BFS
    heuristic = [[0 for x in range(vertical_axis + 1)] for y in range(horizontal_axis + 1)]
    bfsVisted = [[0 for x in range(horizontal_axis + 1)] for y in range(vertical_axis + 1)]
    for i in range(0, len(array)):
        for j in range(0, len(array[i])):
            bfsVisted[i][j] = array[i][j]
    q = Queue()
    q.put(g)
    bfsVisted[s.y][s.x] = 0
    bfsVisted[g.y][g.x] = 1

    while not q.empty():
        u = q.get()
        neighbor = getNeighborNode(u)
        for v in neighbor:
            if bfsVisted[v.y][v.x] == 0:
                bfsVisted[v.y][v.x] = 1
                q.put(v)
                heuristic[v.x][v.y] = heuristic[u.x][u.y] + 1

    # tìm hoán vị có tổng phí đường đi thấp nhất
    permutes = list(permutations(array_pickup_point))
    shortestCost = 100000
    shortestPath = list()
    for permute in permutes:
        listPermuate = list(permute)
        listPermuate.insert(0, s)
        listPermuate.append(g)
        sum = 0
        for i in range(0, len(listPermuate) - 1):
            first = listPermuate[i]
            second = listPermuate[i + 1]
            sum += abs(heuristic[first.x][first.y] - heuristic[second.x][second.y] + 1)
        if shortestCost > sum:
            shortestCost = sum
            shortestPath = listPermuate

    result = list()

    for i in range(0, len(shortestPath) - 1):
        first = shortestPath[i]
        second = shortestPath[i + 1]
        path = BFS_2Point(first, second)
        result.append(path)
    return (result, shortestPath)


if option < 5:
    if option == 1:
        path = BFS()
    elif option == 2:
        path = DFS()
    elif option == 3:
        path = dijkstra()
    elif option == 4:
        path = aStar()
    previousNode = g
    totalCost = 0
    while True:
        if previousNode.x == -1 and previousNode.y == -1:
            print (totalCost)
            break
        w.create_rectangle((previousNode.x + 1) * p, (previousNode.y + 1) * p, (previousNode.x + 1) * p + p,
                           (previousNode.y + 1) * p + p, fill="black")
        totalCost = totalCost + 1
        previousNode = path[previousNode.x][previousNode.y]

if option == 5:
    (paths,shortestPath) = pathFindingWithVistedPoint()
    for i in range(0,len(shortestPath)-1):
        path = paths[i]
        first = shortestPath[i]
        second = shortestPath[i + 1]
        previousNode = second
        totalCost = 0
        while True:
            if previousNode.x == -1 and previousNode.y == -1:
                print (totalCost)
                break
            w.create_rectangle((previousNode.x + 1) * p, (previousNode.y + 1) * p, (previousNode.x + 1) * p + p,
                           (previousNode.y + 1) * p + p, fill="black")
            totalCost = totalCost + 1;
            previousNode = path[previousNode.x][previousNode.y]





for i in range(len(array_pickup_point)):
    w.create_rectangle((array_pickup_point[i].x + 1) * p, (array_pickup_point[i].y + 1) * p,
                       (array_pickup_point[i].x + 1) * p + p, (array_pickup_point[i].y + 1) * p + p, fill='blue4')
    w.create_text((array_pickup_point[i].x + 1) * p + p / 2, (array_pickup_point[i].y + 1) * p + p / 2, text='P',
                  fill='white', font='Impact')
w.create_rectangle((s.x + 1) * p, (s.y + 1) * p, (s.x + 1) * p + p, (s.y + 1) * p + p, fill='blue4')
w.create_text((s.x + 1) * p + p / 2, (s.y + 1) * p + p / 2, text='S', fill='white', font='Impact')
w.create_rectangle((g.x + 1) * p, (g.y + 1) * p, (g.x + 1) * p + p, (g.y + 1) * p + p, fill='blue4')
w.create_text((g.x + 1) * p + p / 2, (g.y + 1) * p + p / 2, text='G', fill='white', font='Impact')
mainloop()
