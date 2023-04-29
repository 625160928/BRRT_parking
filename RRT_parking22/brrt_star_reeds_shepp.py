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
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.init_node_list))
            rnd = self.get_random_node()
            # print(rnd.x,rnd.y,rnd.yaw)
            # if i==100:
            #     rnd=copy.deepcopy(self.end)

            nearest_ind = self.get_nearest_node_index(self.init_node_list, rnd)
            new_node = self.steer(self.init_node_list[nearest_ind], rnd)
            if new_node==None:
                # print("无法与附近的点相连 ",self.node_list[nearest_ind], rnd)
                continue
            if self.check_collision_node(
                    new_node):
                near_indexes = self.find_near_nodes(new_node)

                node_with_updated_parent = self.choose_parent(new_node, near_indexes)

                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_indexes)
                    self.init_node_list.append(node_with_updated_parent)
                    new_node=node_with_updated_parent
                else:
                    self.init_node_list.append(new_node)


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
