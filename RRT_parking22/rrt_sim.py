import math

from hy_src.env_base import env_base
from hy_src.hybrid_astar import hybrid_astar
from hy_src.grid_graph import grid_graph
from pathlib import Path
import numpy as np
import cProfile

from rrt import  RRT
from rrt_star import RRTStar
from rrt_star_reeds_shepp import RRTStarReedsShepp

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
    world_map = str(cur_path / 'map_image' / 'map4.png')
    # world_map = str(cur_path / 'map_image' / 'map_100_100.png')

    env0 = env_base(world_path, world_map)
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
    rrt_star_reeds_shepp = RRTStarReedsShepp(
        start=start_position,
        goal=end_position,
        obstacle_list=obstacleList,
        rand_area=search_area,
        max_iter=1,
        connect_circle_dist=50.0,
        robot_radius=collision_r,
        sim_env=env0,
        grid=grid)


    show_animation =False


    path_list  = rrt_star_reeds_shepp.planning(animation=show_animation,search_until_max_iter=False)



    #RRT path to reformate
    path_list =  path_rrt_to_astar(path_list)


    for path_point in  path_list:
        print(path_point[0][0],path_point[1][0],path_point[2][0],path_point[3][0])

    env0.world.path_plot(path_list, path_color='r', show_point=False)
    env0.render(0.1)
    for point in path_list:
        # env0.world.com_cla()
        env0.car.update_state(point)
        env0.world.path_plot(path_list, path_color='r', show_point=False)
        env0.render(0.001)
    env0.show()

if __name__ == '__main__':
    main()
