Dijkstra算法本身是基于贪心，广度优先，动态优化计算最短距离，最好事先了解一下Dijkstra算法本质上是什么，然后再看在路径规划上的应用比较好。这里贴出来Dijkstra算法介绍的知乎文章[Dijkstra算法详解 通俗易懂 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/338414118)，这里就不做讲解。

## 示例如下

![](https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/gif/Dijkstra.gif?raw=true)

## 伪代码

![image-20230228174409925](https://github.com/LaurentKian/GraphBed/blob/main/Dweidama.png?raw=true)

## Python代码

```python
"""

Grid based Dijkstra planning

author: Atsushi Sakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math

show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None
        # 栅格的大小
        self.resolution = resolution
        # 机器人的半径
        self.robot_radius = robot_radius
        # 构建栅格地图,传入的是障碍物信息
        self.calc_obstacle_map(ox, oy)
        # 机器人的运动方式
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost  # g(n)
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # 求的就是X方向的索引,也就是说在栅格地图中的相对位置
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0,
                               -1)  # round((position - minp) / self.resolution)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()  # key - value: hash表
        # 这里需要理解一下就是我们将栅格地图中的每一个位置都编一个号,从第零行,y=0~y=最大值,一直到最大行,例如下面的例子
        '''
        13 14 15 16 17...
        7 8 9 10 11 12
        1 2 3 4 5 6
        '''
        # 因为open_set是字典结构,所以需要键值对,
        # self.calc_index(start_node)就是起始点的索引
        # 对应的值就是start_node,这样就形成了键值对
        open_set[self.calc_index(start_node)] = start_node
        # 进入循环
        while 1:
            # 取cost最小的节点 key=lambda这个是匿名函数的意思,
            # o: open_set[o].cost就是说按照这个来进行排序
            # 取cost最小的节点的索引
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            # 从open_set中把这个节点取出来
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # 判断是否是终点
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                # 如果是就退出循环
                break

            # 把这个节点从open_set中删除
            del open_set[c_id]

            # 把该点加入到closed_set中
            closed_set[c_id] = current

            # 遍历该点的邻接节点,通过运动方式进行遍历
            for move_x, move_y, move_cost in self.motion:
                # 求得邻接节点的x,y,cost和父节点
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                # 计算对应的索引值
                n_id = self.calc_index(node)

                # 判断这个索引是否已经在closed_set里了，也就是是否已经在路径中了,如果是就跳过下面的步骤
                if n_id in closed_set:
                    continue
                # 判断邻接节点是否是可行的,或者有没有超过范围，如果不可行，那么就直接跳过下面的操作，直接进入下一个循环
                if not self.verify_node(node):
                    continue
                # 如果这个节点还不在open_set里面，那就把他加进去
                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                #  否则
                else:
                    # 其实这一步的目的就是说我有了一个新的临近节点，而原本已经存在与open_set中的节点与这个新的邻接节点距离更近，
                    # 所以我们就需要更新一下对应的距离
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node
        # 计算最后的路径
        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        # 直接计算的就是xy的坐标值
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        # 知道初始节点
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        # 先算有多少个整行,再算剩余的一行
        return node.y * self.x_width + node.x

    def verify_node(self, node):
        # 计算x，y坐标
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        # 判断是否在地图的范围里面
        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False
        # 判断是否在障碍物里面
        if self.obstacle_map[node.x][node.y]:
            return False
        # 如果上面的判断都不是，那么就说明该邻接节点是可以行走的
        return True

    def calc_obstacle_map(self, ox, oy):
        ''' 第1步：构建栅格地图 '''
        # 第一步是获得地图的边界,四个角的位置
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)
        # 计算X方向和Y方向栅格的个数
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # 打印输出栅格的大小
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        # 初始化地图,采用两层的列表来表示,初始化为False说明都还没有设置障碍物
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        # 遍历栅格
        for ix in range(self.x_width):
            # 计算具体的栅格x位置
            x = self.calc_position(ix, self.min_x)
            # 计算具体的栅格的y的位置
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    # 障碍物到栅格的距离
                    d = math.hypot(iox - x, ioy - y)
                    # 膨胀障碍物
                    if d <= self.robot_radius:
                        # 障碍物的栅格地图设置为True
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy具体的行走 方向,cost行走的代价
        # 往x方向(右)走1
        motion = [[1, 0, 1],
                  # 往y方向(上)走1
                  [0, 1, 1],
                  # 左
                  [-1, 0, 1],
                  # 下
                  [0, -1, 1],
                  # 斜下
                  [-1, -1, math.sqrt(2)],
                  # 斜上
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    # start and goal position
    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
```
