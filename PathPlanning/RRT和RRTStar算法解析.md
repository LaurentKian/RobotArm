这一节将会记录一下有关RRT算法，代码是基于[第五讲-RRT算法原理和代码讲解_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1yT4y1T7Eb?p=5&spm_id_from=pageDriver&vd_source=bedb2eb419f5d3f8392dce717a473403)。

RRT和RRT*都是基于采样点的路径规划，都是从空间中随机的选取一个点，并把此点作为树生长的方向。
# 经典RRT
## 经典RRT的逻辑
首先会在空间中随机产生一个样本点，然后在树中寻找一个距离该样本点最近的树节点，然后以树节点和样本点连成直线，根据自己设定的步长，在这条直线的方向产生一个新的树节点，并且把刚才的树节点设置为新节点的父节点。循环遍历，直至新产生的节点距离目标点的位置小于阈值，即找到了可行路径。

产生的效果如下图：
![](https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/RRT_2D.gif?raw=true)

## 注意点
1.在经典RRT中为了提高路径生成速度，我们往往会有概率的产生一个偏向目标节点的随机点，这样会引导树的生长方向。但需要注意的是，我们不能把这个概率调的太高，如果目标点周围障碍物比较多，就会导致一直因为路径朝向终点生长，持续碰撞障碍物而导致无法生成有效地路径，反而得不偿失。
2.经典RRT产生的路径一般不是最优的，并且没有后续优化。

##伪代码
![](https://img2023.cnblogs.com/blog/3021331/202302/3021331-20230228104710723-1175834241.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/600)

## Python代码

~~~python
import copy
import math
import random
import time

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np

show_animation = True


class RRT:
    # 通过面向对象的思路来进行编码
    # 创建一个对象，对应的参数
    def __init__(self, obstacleList, randArea,
                 expandDis=2.0, goalSampleRate=10, maxIter=200):

        self.start = None
        self.goal = None
        # 最小采样值=最小采样范围
        self.min_rand = randArea[0]
        # 最大采样值=最大采样范围
        self.max_rand = randArea[1]
        # expandDis 采样步长
        self.expand_dis = expandDis
        # goalSampleRate 目标采样率,就是说朝着目标方向生长的概率,这里设置的是10%
        self.goal_sample_rate = goalSampleRate
        # maxIter 最大迭代次数
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        # node_list 这个是用来存放RRT树上的节点的
        self.node_list = None

    def rrt_planning(self, start, goal, animation=True):
        # 计时
        start_time = time.time()
        # 起点和终点
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        # 起始节点先加进来
        self.node_list = [self.start]
        # 路径是空的
        path = None

        # 开始进行循环
        for i in range(self.max_iter):
            # 首先会进行一个采样
            rnd = self.sample()
            # 找到离采样点最近的那个节点对应的下标
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            # 找到距离最近的点
            nearestNode = self.node_list[n_ind]

            # 我们现在知道了产生的随机采样点和距离他最近的点,那么我们就可以求出两点连成直线所对应的角度,就是用的atan2方法
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            # 根据已知的数据我们就可以生成一个新的节点
            newNode = self.get_new_node(theta, n_ind, nearestNode)
            # 检查路径是否有碰撞
            noCollision = self.check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            # 如果没有碰撞
            if noCollision:
                # 加入到树中
                self.node_list.append(newNode)
                if animation:
                    self.draw_graph(newNode, path)
                # 判断是否在终点附近
                if self.is_near_goal(newNode):
                    # 如果在终点的附近，那么就检测是否与障碍物发生碰撞，如果发生了碰撞，那么就直接进入下一个循环，继续迭代
                    if self.check_segment_collision(newNode.x, newNode.y,
                                                    self.goal.x, self.goal.y):
                        # 没有发生碰撞，找到最后一个节点的下标
                        lastIndex = len(self.node_list) - 1
                        # 寻找相应的路径
                        path = self.get_final_course(lastIndex)
                        pathLen = self.get_path_len(path)
                        print("迭代次数{}".format(i))
                        print("current path length: {}, It costs {} s".format(pathLen, time.time() - start_time))

                        if animation:
                            self.draw_graph(newNode, path)
                        return path


    def sample(self):
        # random.randint(0, 100)产生一个0-100的随机数,如果它比目标采样率大的化,那么在空间中随机找一个点
        if random.randint(0, 100) > self.goal_sample_rate:
            # 产生一个处于最大值和最小值之间的随机数
            rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        else:  # goal point sampling
            # 否则直接返回终点的坐标就行了
            rnd = [self.goal.x, self.goal.y]
        return rnd

    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2
                  for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)
        return pathLen

    @staticmethod
    def line_cost(node1, node2):
        """
        计算两个节点之间的距离
        :param node1: 节点1
        :param node2: 节点2
        :return: 节点距离的平方
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    # @staticmethod 的意思就是可以不需要对象就可以调用它的方法,也可以叫他静态方法
    @staticmethod
    # 返回最小值对应的下标
    def get_nearest_list_index(nodes, rnd):
        # 计算node与采样点的距离
        dList = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        # copy.deepcopy()是深拷贝，会拷贝对象及其子对象，哪怕以后对其有改动，也不会影响其第一次的拷贝。
        newNode = copy.deepcopy(nearestNode)
        # 计算出对应的xy坐标
        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)
        # 扩展的距离,累加在一起就是路径的距离
        newNode.cost += self.expand_dis
        # 新节点的父节点
        newNode.parent = n_ind
        return newNode

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        '''
        计算点到直线的距离
        :param v: 线段的起始点
        :param w:
        :param p: 线段外的点
        :return: 点到直线距离的平方
        '''
        # Return minimum distance between line segment vw and point p
        # 如果说线段的起始点和终止点位于同一个位置,那么点到直线的距离就是v到p的距离
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)


    def check_segment_collision(self, x1, y1, x2, y2):
        """
        用于检查是否与障碍物发生了碰撞
        :param x1: 线段的起始坐标和终止坐标
        :param y1:
        :param x2:
        :param y2:
        :return:
        """
        # 遍历所有的障碍物
        for (ox, oy, size) in self.obstacle_list:
            # dd 点到直线的距离的平方
            dd = self.distance_squared_point_to_segment(
                np.array([x1, y1]),
                np.array([x2, y2]),
                np.array([ox, oy]))
            # 如果是小于圆的半径的化，就说明发生了碰撞
            if dd <= size ** 2:
                return False  # collision
        return True

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        # 先把终点放进来
        path = [[self.goal.x, self.goal.y]]
        # 找点对应的父节点，如果是None那就说明找到了起点
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        #     加入最后的起始点
        path.append([self.start.x, self.start.y])
        return path




    def draw_graph(self, rnd=None, path=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [
                        node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            # self.plot_circle(ox, oy, size)
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")

        if path is not None:
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.axis([-2, 18, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt planning")

    # create obstacles
    # 创建障碍物,坐标+半径
    # obstacleList = [
    #     (3, 3, 1.5),
    #     (12, 2, 3),
    #     (3, 9, 2),
    #     (9, 11, 2),
    # ]

    obstacleList = [
        (3, 3, 1.5),
        (12, 2, 3),
        (3, 9, 2),
        (9, 11, 2),
        (15, 8, 2.5),
        (12, 10, 1.5),
    ]

    # obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
    #                 (9, 5, 2), (8, 10, 1)]

    # Set params
    # randArea 随机采样的范围, maxIter 最大迭代周期
    rrt = RRT(randArea=[-2, 18], obstacleList=obstacleList, maxIter=200)
    path = rrt.rrt_planning(start=[0, 0], goal=[15, 12], animation=show_animation)
    print("Done!!")

    if show_animation and path:
        plt.show()


if __name__ == '__main__':
    main()
~~~



# RRT*

RRT*在RRT的基础上进行了父节比较，确保选到在指定范围内距离根节点最近的父节点。同时该算法还增加了路径优化，并不是在第一次找到可行路径后就停止遍历，而是一直到指定迭代次数才停止。在此过程中只要由新的可行路径，并且距离比上一条可行路径要短，那么就更新路径。理论上讲该算法得到的路径是渐进最优的，如果迭代次数无限多，得到的路径就无限接近最优路径

实例如下
![](https://github.com/LaurentKian/GraphBed/blob/main/GIF%202023-2-28%2011-08-32.gif?raw=true)

## 伪代码
![](https://img2023.cnblogs.com/blog/3021331/202302/3021331-20230228104243756-201484714.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/600)

## Python代码

~~~Python
import copy
import math
import random
import time

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np

show_animation = True


class RRT:
    # 通过面向对象的思路来进行编码
    # 创建一个对象，对应的参数
    def __init__(self, obstacleList, randArea,
                 expandDis=2.0, goalSampleRate=10, maxIter=200):

        self.start = None
        self.goal = None
        # 最小采样值=最小采样范围
        self.min_rand = randArea[0]
        # 最大采样值=最大采样范围
        self.max_rand = randArea[1]
        # expandDis 采样步长
        self.expand_dis = expandDis
        # goalSampleRate 目标采样率,就是说朝着目标方向生长的概率,这里设置的是10%
        self.goal_sample_rate = goalSampleRate
        # maxIter 最大迭代次数
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        # node_list 这个是用来存放RRT树上的节点的
        self.node_list = None

    # RRT*
    def rrt_star_planning(self, start, goal, animation=True):
        start_time = time.time()
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        # 先把起点放入到一棵树中
        self.node_list = [self.start]
        path = None
        # 初始化一个路径长度为无穷大
        lastPathLength = float('inf')

        for i in range(self.max_iter):
            # 和RRT一样，先采样点
            rnd = self.sample()
            # 找到对应的最接近的节点的下标
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearestNode = self.node_list[n_ind]

            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.get_new_node(theta, n_ind, nearestNode)

            '''生成新的点,这里是根据距离采样点最近的树节点构建出来的新的节点,
               实际上说,虽然新的节点由这个树节点产生,但是从newNode到初始节点的cost未必是最短的
               所以在这里我们先不盲目的把新节点的父节点=树节点,而是对附近的树节点求距离
               从中选取距离最近的那个节点,如果是原来那个树节点,就选那个,否则就选其他的'''

            noCollision = self.check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            # 如果没有碰到障碍物
            if noCollision:
                # 找到附近的节点
                nearInds = self.find_near_nodes(newNode)
                # 这时候的newNode已经拥有了明确的父节点
                newNode = self.choose_parent(newNode, nearInds)
                # 新的节点已经更新完了,这样就可以加入到树里面
                self.node_list.append(newNode)
                # 重新连接原本的节点
                self.rewire(newNode, nearInds)
                # 仿真画图
                if animation:
                    self.draw_graph(newNode, path)
                #如果接近目标节点
                if self.is_near_goal(newNode):
                    if self.check_segment_collision(newNode.x, newNode.y,
                                                    self.goal.x, self.goal.y):
                        lastIndex = len(self.node_list) - 1
                        # 找到相应的路径
                        tempPath = self.get_final_course(lastIndex)
                        # 路径的长度
                        tempPathLen = self.get_path_len(tempPath)
                        # 如果这个条找到的路径比原本的小，那我们就更新一下路径
                        if lastPathLength > tempPathLen:
                            path = tempPath
                            lastPathLength = tempPathLen
                            print(
                                "current path length: {}, It costs {} s".format(tempPathLen, time.time() - start_time))

        return path


    def sample(self):
        # random.randint(0, 100)产生一个0-100的随机数,如果它比目标采样率大的化,那么在空间中随机找一个点
        if random.randint(0, 100) > self.goal_sample_rate:
            # 产生一个处于最大值和最小值之间的随机数
            rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        else:  # goal point sampling
            # 否则直接返回终点的坐标就行了
            rnd = [self.goal.x, self.goal.y]
        return rnd

    def choose_parent(self, newNode, nearInds):
        # 没有合适的候选节点，直接就返回newNode就可以，不需要后续更换父节点
        if len(nearInds) == 0:
            return newNode

        dList = []
        # 开始遍历每一个相近的节点
        for i in nearInds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            # math.hypot返回参数的欧几里得范数
            # 欧几里得范数趋势就是所有元素平方和开平方，对于两个点来说那就是两点之间的距离
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            # 检测是否碰到障碍物
            if self.check_collision(self.node_list[i], theta, d):
                # 没有碰撞就记录一下,以这个新的节点为父节点对应的到起始点的距离
                dList.append(self.node_list[i].cost + d)
            else:
                # 有碰撞就记录成无穷大
                dList.append(float('inf'))
        # 选路径最小的那个点
        minCost = min(dList)
        # 找到对应的节点的索引是哪一个
        minInd = nearInds[dList.index(minCost)]
        #如果最小的cost都是无穷大,那么就说明所有的临近点连成的线都是与障碍物碰撞的,那也就没有必要更换父节点
        if minCost == float('inf'):
            print("min cost is inf")
            return newNode
        # 不是无穷大,这个时候更新一下对应的cost
        newNode.cost = minCost
        # 并且把对应的父节点也更新一下
        newNode.parent = minInd

        return newNode

    def find_near_nodes(self, newNode):
        # 首先获得树里面的节点个数
        n_node = len(self.node_list)
        # 对应的搜寻附近节点的半径，是根据树的节点的个数动态变化的，
        # log(n_node) / n_node) 决定了我们的策略是随着搜索越往后进行，搜索的范围越小，这样可以减少遍历的次数
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        # 距离的平方
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2
                  for node in self.node_list]
        # 遍历比较距离的平方，存储对应的下标
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds




    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    @staticmethod
    def line_cost(node1, node2):
        """
        计算两个节点之间的距离
        :param node1: 节点1
        :param node2: 节点2
        :return: 节点距离的平方
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    # @staticmethod 的意思就是可以不需要对象就可以调用它的方法,也可以叫他静态方法
    @staticmethod
    # 返回最小值对应的下标
    def get_nearest_list_index(nodes, rnd):
        # 计算node与采样点的距离
        dList = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        # copy.deepcopy()是深拷贝，会拷贝对象及其子对象，哪怕以后对其有改动，也不会影响其第一次的拷贝。
        newNode = copy.deepcopy(nearestNode)
        # 计算出对应的xy坐标
        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)
        # 扩展的距离,累加在一起就是路径的距离
        newNode.cost += self.expand_dis
        # 新节点的父节点
        newNode.parent = n_ind
        return newNode

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    def rewire(self, newNode, nearInds):
        # 当前树上节点的个数
        n_node = len(self.node_list)
        # 遍历newNode的圆圈中对应的邻近的节点
        for i in nearInds:
            nearNode = self.node_list[i]
            # 计算两个节点的距离
            d = math.sqrt((nearNode.x - newNode.x) ** 2
                          + (nearNode.y - newNode.y) ** 2)
            # 选择newNode作为父节点之后对应的cost
            s_cost = newNode.cost + d
            '''这时候节点以newNode作为父节点那么，比该节点的新的到根节点的距离与原来到根节点的距离
            如果新的小于旧的那么我们就把该节点的父节点更新为newNode，否则保持原状
            起始在这个临近的列表中也存在newNode的父节点，但是不用担心，因为如果newNode变成了父节点，
            那么他们两个互为父节点，他到根节点的距离就是无限大，不影响最后的结果
            '''
            if nearNode.cost > s_cost:
                theta = math.atan2(newNode.y - nearNode.y,
                                   newNode.x - nearNode.x)
                if self.check_collision(nearNode, theta, d):
                    # 因为newNode是在树的最后一个节点，更新父节点
                    nearNode.parent = n_node - 1
                    # 更新距离
                    nearNode.cost = s_cost

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        '''
        计算点到直线的距离
        :param v: 线段的起始点
        :param w:
        :param p: 线段外的点
        :return: 点到直线距离的平方
        '''
        # Return minimum distance between line segment vw and point p
        # 如果说线段的起始点和终止点位于同一个位置,那么点到直线的距离就是v到p的距离
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)


    def check_segment_collision(self, x1, y1, x2, y2):
        """
        用于检查是否与障碍物发生了碰撞
        :param x1: 线段的起始坐标和终止坐标
        :param y1:
        :param x2:
        :param y2:
        :return:
        """
        # 遍历所有的障碍物
        for (ox, oy, size) in self.obstacle_list:
            # dd 点到直线的距离的平方
            dd = self.distance_squared_point_to_segment(
                np.array([x1, y1]),
                np.array([x2, y2]),
                np.array([ox, oy]))
            # 如果是小于圆的半径的化，就说明发生了碰撞
            if dd <= size ** 2:
                return False  # collision
        return True

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        # 先把终点放进来
        path = [[self.goal.x, self.goal.y]]
        # 找点对应的父节点，如果是None那就说明找到了起点
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        #     加入最后的起始点
        path.append([self.start.x, self.start.y])
        return path


    def draw_graph(self, rnd=None, path=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [
                        node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            # self.plot_circle(ox, oy, size)
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")

        if path is not None:
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.axis([-2, 18, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt planning")

    # create obstacles
    # 创建障碍物,坐标+半径
    # obstacleList = [
    #     (3, 3, 1.5),
    #     (12, 2, 3),
    #     (3, 9, 2),
    #     (9, 11, 2),
    # ]

    obstacleList = [
        (3, 3, 1.5),
        (12, 2, 3),
        (3, 9, 2),
        (9, 11, 2),
        (15, 8, 2.5),
        # (12, 10, 1.5),
    ]

    # obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
    #                 (9, 5, 2), (8, 10, 1)]

    # Set params
    # randArea 随机采样的范围, maxIter 最大迭代周期
    rrt = RRT(randArea=[-2, 18], obstacleList=obstacleList, maxIter=300)

    path = rrt.rrt_star_planning(start=[0, 0], goal=[15, 12], animation=show_animation)

    print("Done!!")

    if show_animation and path:
        plt.show()


if __name__ == '__main__':
    main()

~~~