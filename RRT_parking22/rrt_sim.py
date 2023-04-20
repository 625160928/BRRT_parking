import math

from hy_src.env_base import env_base
from hy_src.hybrid_astar import hybrid_astar
from hy_src.grid_graph import grid_graph
from pathlib import Path
import numpy as np
import cProfile

from rrt import  RRT
from rrt_star import RRTStar
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

def main(show_animation=True):

    cur_path = Path(__file__).parent
    world_path = str(cur_path / 'hy_astar_world.yaml')
    reeds_lookup_path = str(cur_path / 'reeds_lookup.npy')
    world_map = str(cur_path / 'map_image' / 'map_100_100_4.png')

    env0 = env_base(world_path, world_map)
    env0.initialization()

    grid = grid_graph(env0.map_matrix, xy_reso=env0.xy_reso, yaw_reso=env0.yaw_reso)
    path_list=[]


    # show_animation =False
    """
    hybrid astar
    """
    # hy_astar = hybrid_astar(grid, env0.car.shape, step_size=2, reeds_size=1,
    #                         min_radius=3, test_plot=env0, lookup_path=reeds_lookup_path)
    # # print('env0.car.state ',env0.car.state[0][0],env0.car.state[1][0])
    # # print('env0.car.goal ',env0.car.goal)
    # # print('env0.car.shape ',env0.car.shape)
    # # print(env0.width,env0.height)
    # path_list = hy_astar.hy_astar_search(env0.car.state, env0.car.goal, show_process=show_animation)

    """
    param for RRT
    """

    obstacleList=[]
    start_position=[env0.car.state[0][0],env0.car.state[1][0],env0.car.state[2][0],env0.car.state[3][0]]
    end_position=[env0.car.goal[0][0],env0.car.goal[1][0],env0.car.goal[2][0],env0.car.goal[3][0]]
    collision_r=  math.sqrt(env0.car.shape[0]*env0.car.shape[0]+env0.car.shape[1]*env0.car.shape[1])/2
    search_area=[0,env0.height]
    # search_area=[env0.width,env0.height]
    # print(start_position,end_position,collision_r)

    """
    RRT
    """
    # rrt = RRT(
    #     start=start_position,
    #     goal=end_position,
    #     rand_area=search_area,
    #     obstacle_list=obstacleList,
    #     # play_area=[0, 10, 0, 14]
    #     max_iter=5000,
    #     robot_radius=collision_r,
    #     sim_env=env0,
    #     grid=grid
    #     )
    #
    # path_list = rrt.planning(animation=show_animation)

    """
    RRT*
    """
    # Set Initial parameters
    rrt_star = RRTStar(
        start=start_position,
        goal=end_position,
        rand_area=search_area,
        obstacle_list=obstacleList,
        max_iter=5000,
        expand_dis=1,
        robot_radius=collision_r,
        sim_env=env0,
        grid=grid)
    path_list = rrt_star.planning(animation=show_animation)

    path_list =  path_rrt_to_astar(path_list)


    for path_point in  path_list:
        print(path_point)

    env0.world.path_plot(path_list, path_color='r', show_point=False)
    env0.render(0.1)
    for point in path_list:
        # env0.world.com_cla()
        env0.car.update_state(point)
        env0.world.path_plot(path_list, path_color='r', show_point=False)
        env0.render(0.1)
    env0.show()

if __name__ == '__main__':
    main()
