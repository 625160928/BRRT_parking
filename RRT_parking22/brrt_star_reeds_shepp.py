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
from rrt_star_reeds_shepp import *
show_animation = True
#https://www.cnblogs.com/21207-iHome/p/7210543.html
class BRRTStarReedsShepp(RRTStarReedsShepp):
    """
    Class for RRT star planning with Reeds Shepp path
    """


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
        super().__init__(start, goal, obstacle_list, rand_area, max_iter,connect_circle_dist,
                         robot_radius=robot_radius, goal_sample_rate=goal_sample_rate,sim_env=sim_env, grid=grid,
                 path_collision_check_mode=path_collision_check_mode)

    def rewire(self, new_node, near_inds,NodeLists):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = NodeLists[i]
            while near_node!=None:
                edge_node = self.steer(new_node, near_node)
                if not edge_node:
                    near_node=None
                    continue
                edge_node.cost = self.calc_new_cost(new_node, near_node)

                no_collision = self.check_collision_node(
                    edge_node)
                improved_cost = near_node.cost > edge_node.cost

                if no_collision and improved_cost:
                    parent_node=near_node.parent
                    near_node.x = edge_node.x
                    near_node.y = edge_node.y
                    near_node.cost = edge_node.cost
                    near_node.path_x = edge_node.path_x
                    near_node.path_y = edge_node.path_y
                    near_node.parent = edge_node.parent
                    self.propagate_cost_to_leaves(new_node)
                    near_node=parent_node
                else:
                    near_node=None


    def find_near_nodes(self, new_node,NodeLists):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(NodeLists) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        r=r**2
        dist_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
                     for node in NodeLists]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r]
        return near_inds


    def choose_parent(self, new_node, near_inds,NodeLists):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """


        if not near_inds:
            new_node = self.get_best_father(new_node)
            return new_node

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = NodeLists[i]
            t_node = self.steer(near_node, new_node)
            # if t_node and self.check_collision(
            #         t_node, self.obstacle_list, self.robot_radius):
            if t_node and self.check_collision_node(
                    t_node):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(NodeLists[min_ind], new_node)
        new_node.cost = min_cost
        if new_node.parent==None:
            return new_node

        # print('be ',new_node.parent.x,new_node.parent.y)

        new_node=self.get_best_father(new_node)

        return new_node

    def update_node_into_tree(self, new_node, small_tree):
        near_indexes = self.find_near_nodes(new_node, small_tree)

        node_with_updated_parent = self.choose_parent(new_node, near_indexes, small_tree)

        if node_with_updated_parent:
            self.rewire(node_with_updated_parent, near_indexes,small_tree)
            small_tree.append(node_with_updated_parent)
            new_node=node_with_updated_parent
        else:
            small_tree.append(new_node)
        return new_node

    def generate_final_course_node(self, node):
        path = []
        # path = [[self.end.x, self.end.y, self.end.yaw]]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        # path.append([self.start.x, self.start.y, self.start.yaw])
        return path


    def planning_small_large_once(self, small_tree, large_tree, header, animation=False):

        rnd = self.get_random_node(goal_rate=-1)

        nearest_small_ind = self.get_nearest_node_index(small_tree, rnd)
        new_small_tree_node = self.steer(small_tree[nearest_small_ind], rnd)
        if new_small_tree_node == None:
            return None
        if self.check_collision_node(
                new_small_tree_node):
            new_small_tree_node = self.update_node_into_tree(new_small_tree_node, small_tree)

            if animation and new_small_tree_node != None:  # and i % 5 == 0

                path_list = node_path_to_path_list(new_small_tree_node)
                self.sim_env.world.path_plot(path_list, path_color='black')
                self.sim_env.world.point_arrow_plot(node_to_point(new_small_tree_node), length=1)
                self.sim_env.world.pause(0.00001)

            # brrt part

            nearest_large_ind = self.get_nearest_node_index(large_tree, new_small_tree_node)
            new_large_tree_node = self.steer(large_tree[nearest_large_ind], new_small_tree_node)

            if new_large_tree_node == None:
                # print("无法与附近的点相连 ",self.node_list[nearest_ind], rnd)
                return None

            if self.check_collision_node(
                    new_large_tree_node):
                new_large_tree_node = self.update_node_into_tree(new_large_tree_node, large_tree)

                if animation and new_large_tree_node != None:  # and i % 5 == 0

                    path_list = node_path_to_path_list(new_large_tree_node)
                    print()
                    self.sim_env.world.path_plot(path_list, path_color='blue')
                    self.sim_env.world.point_arrow_plot(node_to_point(new_small_tree_node), length=1)
                    self.sim_env.world.pause(0.00001)

                if self.node_eq(new_large_tree_node, new_small_tree_node):
                    # for p in self.generate_final_course_node(new_large_tree_node):
                    #     print(p)
                    # print()
                    # print('===============')
                    # for p in self.generate_final_course_node(new_small_tree_node):
                    #     print(p)
                    # print(type(self.generate_final_course_node(new_large_tree_node)))
                    if header == 'end':

                        p1 =self.generate_final_course_node(new_large_tree_node)
                        p1.reverse()
                        path = p1+self.generate_final_course_node(new_small_tree_node)
                    else:
                        p1 = self.generate_final_course_node(new_small_tree_node)
                        p1.reverse()
                        path = p1+self.generate_final_course_node(new_large_tree_node)
                    return path
        return None

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

        self.init_node_list = [self.start]
        self.end_node_list = [self.end]



        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.init_node_list),len(self.end_node_list))
            if len(self.init_node_list)>len(self.end_node_list):
                path=self.planning_small_large_once(self.end_node_list, self.init_node_list, 'end', animation=animation)
            else:
                path=self.planning_small_large_once(self.init_node_list, self.end_node_list, 'start', animation=animation)
            if path!=None:
                return path




        print("reached max iteration")

        print("Cannot find path")

        return None

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

    rrt_star_reeds_shepp = BRRTStarReedsShepp(start, goal,
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
