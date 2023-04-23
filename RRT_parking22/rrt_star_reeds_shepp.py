"""

Path planning Sample Code with RRT with Reeds-Shepp path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import sys
import pathlib
import matplotlib.pyplot as plt
import numpy as np
# sys.path.append(str(pathlib.Path(__file__).parent.parent))

import reeds_shepp_path_planning
from rrt_star import RRTStar

show_animation = True

def node_to_point(node):
    return np.array([[node.x],[node.y],[node.yaw]])

def node_path_to_path_list(node):
    px=node.path_x
    py=node.path_y
    pyaw=node.path_yaw
    path=[]
    for i in range(len(px)):
        path.append([[px[i]],[py[i]],[pyaw[i]]])
    return np.array(path)

class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200,
                 connect_circle_dist=50.0,
                 robot_radius=0.0,

                 goal_sample_rate=20,
                 sim_env=None,
                 grid=None,
                 path_collision_check_mode='default'
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.robot_radius = robot_radius
        self.goal_sample_rate = goal_sample_rate

        self.curvature = 1.0
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5
        self.sim_env=sim_env
        self.grid=grid
        self.path_collision_check_mode=path_collision_check_mode

        self._collision_check_times=0
        self._safe_collision_check_times=0

    def planning(self, animation=True, search_until_max_iter=True):
        """
        planning

        animation: flag for animation on or off
        """

        if self.check_collision_node(self.start) == False:
            print('start point collision')
            return None

        if self.check_collision_node(self.end) == False:
            print('end point collision')
            return None

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            # if i==100:
            #     rnd=copy.deepcopy(self.end)

            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)
            if new_node==None:
                # print("无法与附近的点相连 ",self.node_list[nearest_ind], rnd)
                continue
            if self.check_collision_node(
                    new_node):
                near_indexes = self.find_near_nodes(new_node)

                node_with_updated_parent = self.choose_parent(new_node, near_indexes)

                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_indexes)
                    self.node_list.append(node_with_updated_parent)
                    new_node=node_with_updated_parent
                else:
                    self.node_list.append(new_node)


                # self.try_goal_path(new_node)

            if animation and new_node!=None: # and i % 5 == 0

                path_list=node_path_to_path_list(new_node)
                self.sim_env.world.path_plot(path_list,path_color='black')
                self.sim_env.world.point_arrow_plot(node_to_point(new_node),length=1)
                self.sim_env.world.pause(0.00001)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision_node(
                new_node):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)


    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        if not px:
            return None

        if self.check_collision_pose_list(px, py, pyaw,order=self.path_collision_check_mode)==False:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):


        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(-math.pi, math.pi)
                            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y,self.end.yaw)
        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        # print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        # print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        # print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def main(max_iter=100):
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 8, 1),
        (4, 10, 1),
        (6, 5, 1),
        (7, 5, 1),
        (8, 6, 1),
        (8, 8, 1),
        (8, 10, 1)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [6.0, 7.0, np.deg2rad(90.0)]

    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             [-2.0, 15.0], max_iter=max_iter)
    path = rrt_star_reeds_shepp.planning(animation=show_animation)

    # Draw final path
    if path and show_animation:  # pragma: no cover
        rrt_star_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
