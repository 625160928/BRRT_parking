import math
import random
import numpy as np
import cProfile
import time

from hy_src.env_base import env_base
from hy_src.hybrid_astar import hybrid_astar
from hy_src.grid_graph import grid_graph
from pathlib import Path

from rrt import  RRT
from rrt_star import RRTStar
from rrt_star_reeds_shepp import RRTStarReedsShepp
from brrt_star_reeds_shepp import BRRTStarReedsShepp
from rrt_sim import *


def gen_anylize_once(path_collision_check_mode,star_tree_sample_method):
    show_animation = False
    cur_path = Path(__file__).parent
    world_path = str(cur_path / 'parking.yaml')
    world_map = str(cur_path / 'map_image' / 'map_parking.png')

    env0 = env_base(world_path, world_map,draw=show_animation)
    env0.initialization()

    grid = grid_graph(env0.map_matrix, xy_reso=env0.xy_reso, yaw_reso=env0.yaw_reso)

    """
    param for RRT
    """

    obstacleList=[]
    start_position=[env0.car.state[0][0],env0.car.state[1][0],env0.car.state[2][0],env0.car.state[3][0]]
    end_position=[env0.car.goal[0][0],env0.car.goal[1][0],env0.car.goal[2][0],env0.car.goal[3][0]]
    collision_r=  math.sqrt(env0.car.shape[0]*env0.car.shape[0]+env0.car.shape[1]*env0.car.shape[1])/2
    search_area=[0,env0.height]
    # search_area=[env0.width,env0.height]


    """
    RRT*-reeds_shepp
    """
    rrt_star_reeds_shepp = BRRTStarReedsShepp(
        start=start_position,
        goal=end_position,
        obstacle_list=obstacleList,
        rand_area=search_area,
        max_iter=5000,
        connect_circle_dist=8.0,
        robot_radius=collision_r,
        sim_env=env0,
        grid=grid,
        # path_collision_check_mode="dichotomy"
        # path_collision_check_mode="hierarchical"
        path_collision_check_mode=path_collision_check_mode,
        star_tree_sample_method= star_tree_sample_method
    )
    rrt_star_reeds_shepp.curvature=1


    # show_animation =False


    start_time=time.time()
    path_list  = rrt_star_reeds_shepp.planning(animation=show_animation,search_until_max_iter=False,log=False)

    end_time=time.time()
    tt=(end_time-start_time)*1000
    tms=(tt)%1000
    ts=(tt-tms)/1000
    point_collision_times,safe_point_collision_times,route_collision_check_times,safe_route_collision_check_times\
        =rrt_star_reeds_shepp.get_collision_check_times()

    len_node=[len(rrt_star_reeds_shepp.init_node_list),len(rrt_star_reeds_shepp.end_node_list)]
    run_times_iter=rrt_star_reeds_shepp.run_times
    try_expand_init=rrt_star_reeds_shepp.try_start_sample_times
    try_expand_init_collision=rrt_star_reeds_shepp.try_start_sample_collision_times


    # print('seed ', seed)
    if show_animation:
        if path_list:
            # RRT path to reformate
            path_list = path_rrt_to_astar(path_list)

            for path_point in  path_list:
                is_save=True
                # is_save=rrt_star_reeds_shepp.check_collision_pose(path_point[0][0],path_point[1][0],path_point[2][0])
                print(path_point[0][0],path_point[1][0],path_point[2][0],path_point[3][0],is_save)

            env0.world.path_plot(path_list, path_color='r', show_point=False)
            env0.render(0.1)
            for point in path_list:
                # env0.world.com_cla()
                env0.car.update_state(point)
                env0.world.path_plot(path_list, path_color='r', show_point=False)
                env0.render(0.001)
            env0.show()
        else:
            print("no route return")

        # print('seed ', seed)

    return point_collision_times,safe_point_collision_times,route_collision_check_times,\
           safe_route_collision_check_times,tt,len_node,run_times_iter,try_expand_init,try_expand_init_collision

def gen_anylize():
    n=30

    random.seed(99)


    # path_collision_check_mode='default'
    path_collision_check_mode="hierarchical"

    star_tree_sample_method='default'
    # star_tree_sample_method='avoid'
    # star_tree_sample_method='limit'

    # for star_tree_sample_method in ['default','avoid','limit']:
    for star_tree_sample_method in ['limit']:
        total_time=0
        total_find_collision_path=0
        total_find_collision_point=0
        total_try_expand_init_point=0
        total_final_expand_init_point=0
        total_final_end_init_point=0
        total_try_expand_init_collision_point=0

        for i in range(n):
            point_collision_times,safe_point_collision_times,route_collision_check_times,\
            safe_route_collision_check_times,tt,len_node,run_times_iter,try_expand_init,try_expand_init_collision\
                =gen_anylize_once( path_collision_check_mode,star_tree_sample_method)

            total_time+=tt
            total_find_collision_path+=route_collision_check_times - safe_route_collision_check_times
            total_find_collision_point +=(point_collision_times - safe_point_collision_times)
            total_try_expand_init_point+=try_expand_init
            total_final_expand_init_point+=len_node[0]
            total_final_end_init_point +=len_node[1]
            total_try_expand_init_collision_point+=try_expand_init_collision


        avg_time=total_time/n

        avg_collision_path=total_find_collision_point/total_find_collision_path
        print('-----------------------')
        print('总运行次数 ',n,' 总时间 ',total_time)
        print('平均运行时间',avg_time,'碰撞顺序算法是：',path_collision_check_mode,' 起点树采样算法 ',star_tree_sample_method)
        print("路径碰撞检测次数 ",total_find_collision_point, ' 路径发生碰撞次数 ',total_find_collision_path
             , ' 平均碰撞检测次数 ', avg_collision_path)
        print('尝试拓展起点树次数 ',total_try_expand_init_point,total_final_expand_init_point,' 生成的起始点能加入树的概率 ',total_final_expand_init_point/total_try_expand_init_point
              ,' 尝试拓展起始点安全概率 ',total_try_expand_init_collision_point/total_try_expand_init_point)
        print("平均拓展节点数量 ",total_final_expand_init_point/n,total_final_end_init_point/n)


if __name__ == '__main__':
    gen_anylize()
