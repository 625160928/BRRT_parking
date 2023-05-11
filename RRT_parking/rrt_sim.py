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

#32724

'''
将rrt格式的路径转化为astar格式
'''
def path_rrt_to_astar(path_list):
    path=[]
    for path_point in path_list :
        if len(path_point)==2:
            x=path_point[0]
            y=path_point[1]
            theta=0
            dir=1
        else:
            x = path_point[0]
            y = path_point[1]
            theta = path_point[2]
            dir = 1
        path.append(np.array([[x],[y],[theta],[dir]]))

    return path

'''
碰撞检测
'''
def collision_check_point(rrt,x,y,theta):
    rectangle = rrt.angular_pos([x, y, theta])
    rect_collision = rrt.grid.check_collision_rectangle(rectangle)
    if rect_collision:
        return False
    return True  # safe

def main(show_animation=True):

    # seed = random.randint(1, 10000000)
    # seed = 5886349
    seed = 588634

    random.seed(seed)

    cur_path = Path(__file__).parent
    world_path = str(cur_path / 'parking.yaml')
    world_map = str(cur_path / 'map_image' / 'map_parking.png')

    # world_path = str(cur_path / 'hy_astar_world.yaml')
    # world_map = str(cur_path / 'map_image' / 'map2.png')
    # world_map = str(cur_path / 'map_image' / 'map_100_100_4.png')
    # world_map = str(cur_path / 'map_image' / 'map_100_100_5.png')

    #起点树采样方法
    star_tree_sample_method='default'
    # star_tree_sample_method='avoid'
    # star_tree_sample_method='limit'
    # star_tree_sample_method='rate_limit'


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
        path_collision_check_mode="hierarchical",
        star_tree_sample_method= star_tree_sample_method
    )
    rrt_star_reeds_shepp.curvature=1




    start_time=time.time()
    path_list  = rrt_star_reeds_shepp.planning(animation=show_animation,search_until_max_iter=False)

    end_time=time.time()
    tt=(end_time-start_time)*1000
    tms=(tt)%1000
    ts=(tt-tms)/1000
    point_collision_times,safe_point_collision_times,route_collision_check_times,safe_route_collision_check_times\
        =rrt_star_reeds_shepp.get_collision_check_times()

    print("规划花费时间 ",ts,' s ',tms,' ms')
    print('生成节点数量 ', len(rrt_star_reeds_shepp.init_node_list))
    print("碰撞检测次数 ",point_collision_times,safe_point_collision_times, ' 发现碰撞次数 ' ,point_collision_times-safe_point_collision_times)
    tmp=route_collision_check_times-safe_route_collision_check_times
    if tmp==0:
        tmp=1
        print('0-> 1')
    print("路径碰撞检测次数 ",route_collision_check_times,safe_route_collision_check_times, ' 路径发生碰撞次数 ' ,
          route_collision_check_times-safe_route_collision_check_times,' 平均碰撞检测次数 ',(point_collision_times-safe_point_collision_times)/(tmp))


    # print('seed ', seed)

    if path_list:
        # RRT path to reformate
        path_list = path_rrt_to_astar(path_list)

        # for path_point in  path_list:
        #     is_save=True
            # is_save=rrt_star_reeds_shepp.check_collision_pose(path_point[0][0],path_point[1][0],path_point[2][0])
            # print(path_point[0][0],path_point[1][0],path_point[2][0],path_point[3][0],is_save)

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

if __name__ == '__main__':
    main()
