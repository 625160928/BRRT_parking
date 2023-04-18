from math import pi, sin, cos, sqrt, atan2
import numpy as np
from ir_sim.global_param import env_param
from ir_sim.util.collision_dection_distance import range_cir_seg, range_seg_seg
from ir_sim.util.util import get_transform, WrapToPi

class lidar2d:
    def __init__(self, robot_state=np.zeros((3, 1)), range_min=0, range_max=10, angle_range = pi, number=36, scan_time=0.1, noise=False, std=0.2, angle_std=0.02, offset=[0, 0, 0], reso=0.05, alpha=0.3, **kwargs) -> None:

        # scan_matrix: (2, sample_num * number)
        # global_scan_matrix: (2, sample_num * number)

        # scan data (refernece: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = - angle_range/ 2
        self.angle_max = angle_range / 2
        self.angle_inc = angle_range / number
        self.scan_time = scan_time
        self.time_inc = (angle_range / (2*pi) ) * scan_time / number # 
        self.range_data = range_max * np.ones(number,)

        # offset
        self.offset = offset

        self.number = number
        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)

        # self.intersections, self.start_ray = self.init_sections() # 
        self.reso = reso
        self.sample_num = int( (self.range_max - self.range_min) / reso )
        self.scan_matrix = self.init_sections()

        if robot_state.shape[0] == 2 :
            robot_state = np.vstack((robot_state, [0]))

        self.robot_state = robot_state

        trans_matirx, rot_matrix = get_transform(robot_state)

        # transform to the global coordinate
        self.global_scan_matrix = rot_matrix @ self.scan_matrix + trans_matirx 
        self.global_intersections = rot_matrix @ self.intersections + trans_matirx 
        self.global_ray = rot_matrix @ self.ray + trans_matirx

        # noise
        self.noise = noise
        self.std = std
        self.angle_std = angle_std   # for obstacle detection

        # for plot
        self.alpha = alpha  # Set the alpha value used for blending,  0-1 range

        self.grid_map = env_param.grid_map

    def init_sections(self):
        # discrete the scan to generate a scan matrix 
        temp_scan_matrix = self.range_max * np.ones((2, self.sample_num, self.number))

        for i, angle in enumerate(self.angle_list):

            cur_range = self.range_min

            for j in range(self.sample_num):  
                cur_range += self.reso              
                temp_scan_matrix[:, j, i] = cur_range * np.array([ cos(angle), sin(angle) ])
                
        self.intersections = temp_scan_matrix[:, -1, :]
        self.ray = temp_scan_matrix[:, 0, :]

        init_scan_matrix = np.reshape(temp_scan_matrix, (2, self.sample_num*self.number))   
        trans_matirx, rot_matrix = lidar2d.transform_matrix(*self.offset)
        scan_matrix = rot_matrix @ init_scan_matrix + trans_matirx

        return scan_matrix

    def step(self, robot_state=np.zeros((3, 1))):
        # calculate the scan range data
        if robot_state.shape[0] == 2 :
            robot_state = np.vstack((robot_state, [0]))

        self.robot_state = robot_state

        trans_matirx, rot_matrix = get_transform(robot_state)
        self.global_scan_matrix = rot_matrix @ self.scan_matrix + trans_matirx
        self.global_ray = rot_matrix @ self.ray + trans_matirx

        Components = env_param.components.copy()
        self.com_list = [com for com in Components if lidar2d.distance(com.center, robot_state[0:2]) >= 0.01]

        closest_index_array = self.ray_casting(self.com_list)

        temp_global_scan_matrix = np.reshape(self.global_scan_matrix, (2, self.sample_num, self.number)) 
        
        for i in range(len(closest_index_array)):   
            self.global_intersections[:, i] = temp_global_scan_matrix[:, closest_index_array[i], i]
            self.range_data[i] = self.range_min + (closest_index_array[i] + 2) * self.reso

        if self.noise:
            self.range_data = self.range_data + np.random.normal(0, self.std, self.range_data.shape)


    def ray_casting(self, com_list):
        # calculate the minimum distance index between global_scan_matrix and obstacles

        index_array = np.ones((1, self.number)) * (self.sample_num - 1)

        # check with map obstacle
        if self.grid_map is not None:
            map_reso = env_param.reso
            # self.global_scan_matrix[:, 0]
            temp_global_index = (self.global_scan_matrix / map_reso).astype(int)
            
            index_x = np.clip(temp_global_index[0, :], 0, self.grid_map.shape[0]-1)
            index_y = np.clip(temp_global_index[1, :], 0, self.grid_map.shape[1]-1)

            global_index = self.grid_map[index_x, index_y] > 50
            collision_matrix = np.reshape(global_index, (self.sample_num, self.number))
            collision_matrix[-1, :] = True
            index = np.argmax(collision_matrix == True, axis = 0) - 1  # find the cloest collision point
            map_index = np.clip(index, 0, self.sample_num)  # be positive

            index_array = np.vstack((index_array, map_index))
            
        # check with objects 
        # if len(com_list) == 0:
        #     closest_index_array = ( (self.sample_num - 2) * np.ones((self.number, )) ).astype(int)
        if len(com_list) != 0:
            
            for i, com in enumerate(com_list):
                temp_collision_matrix = com.collision_check_array(self.global_scan_matrix)  # check the collision of the scan array and 
                collision_matrix = np.reshape(temp_collision_matrix, (self.sample_num, self.number))
                collision_matrix[-1, :] = True  # set the end point true
                index = np.argmax(collision_matrix == True, axis = 0) - 1  # find the cloest collision point
                com_index = np.clip(index, 0, self.sample_num)  # be positive
                index_array = np.vstack((index_array, com_index))

        closest_index_array = np.min(index_array, axis = 0).astype(int)

        return closest_index_array

    def get_LaserScan(self):
        # reference: ros topic -- scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html 
        scan_data = {}
        scan_data['angle_min'] = self.angle_min
        scan_data['angle_max'] = self.angle_max
        scan_data['angle_increment'] = self.angle_inc
        scan_data['time_increment'] = self.time_inc
        scan_data['scan_time'] = self.scan_time
        scan_data['range_min'] = self.range_min
        scan_data['range_max'] = self.range_max
        scan_data['ranges'] = self.range_data
        scan_data['intensities'] = None

        return scan_data

    def get_landmarks(self):

        landmarks = []

        for com in self.com_list:
            if com.landmark:
                dis, radian = lidar2d.relative_position(self.robot_state[0:2], com.center[0:2])
                radian = WrapToPi(radian - self.robot_state[2, 0])
                ang_min = self.angle_min
                ang_max = self.angle_max

                if WrapToPi(radian - ang_min) >= 0 and WrapToPi(ang_max - radian) >= 0 and dis <= self.range_max:
                    if self.noise:
                        dis += np.random.normal(0, self.std)
                        dis = round(np.clip(dis, self.range_min, self.range_max), 2)
                        radian += np.random.normal(0, self.angle_std)
                        radian = round(np.clip(radian, self.angle_min, self.angle_max), 2)
                    landmark = {'id': com.id, 'range': dis, 'angle': radian, 'name': com.name}
                    landmarks.append(landmark)

        return landmarks

    def get_obstacles(self):

        Obstacles = []

        for com in self.com_list:
            
            dis, radian = lidar2d.relative_position(self.robot_state[0:2], com.center[0:2])

            if dis == 0:
                continue

            radian = WrapToPi(radian - self.robot_state[2, 0])
            ang_min = self.angle_min
            ang_max = self.angle_max

            if WrapToPi(radian - ang_min) >= 0 and WrapToPi(ang_max - radian) >= 0 and dis <= self.range_max:
                if self.noise:
                    dis += np.random.normal(0, self.std)
                    dis = round(np.clip(dis, self.range_min, self.range_max), 2)
                    radian += np.random.normal(0, self.angle_std)
                    radian = round(np.clip(radian, self.angle_min, self.angle_max), 2)
                obs = {'id': com.id, 'range': dis, 'angle': radian, 'name': com.name}
                Obstacles.append(obs)

        return Obstacles


    # def init_sections(self):

    #     init_intersections = self.range_max * np.ones((2, self.number))
    #     init_start_ray = self.range_min * np.ones((2, self.number))

    #     for i, angle in enumerate(self.angle_list):
    #         init_intersections[:, i] = self.range_max * np.array([ cos(angle), sin(angle) ])
    #         init_start_ray[:, i] = self.range_min * np.array([ cos(angle), sin(angle) ])
        
    #     trans_matirx, rot_matrix = lidar2d.transform_matrix(*self.offset)

    #     intersections = rot_matrix @ init_intersections + trans_matirx
    #     start_ray = rot_matrix @ init_start_ray + trans_matirx

    #     return intersections, start_ray    

    # def step(self, robot_state=np.zeros((3, 1))):
    #     # calculate the scan range data
    #     trans_matirx, rot_matrix = lidar2d.transform_matrix(*np.squeeze(robot_state))

    #     self.global_intersections = rot_matrix @ self.intersections + trans_matirx
    #     self.global_ray = rot_matrix @ self.start_ray + trans_matirx

    #     Components = env_param.components.copy()

    #     com_list = [com for com in Components if lidar2d.distance(com.center, robot_state[0:2]) >= 0.01]

    #     for i, (start, end) in enumerate(zip(self.global_ray.T, self.global_intersections.T)):
    #         ray = [start, end]

    #         collision_flag, min_int_point, lrange = self.ray_casting(ray, com_list)  
            
    #         if collision_flag:
    #             self.global_intersections[:, i] = min_int_point
    #             self.range_data[i] = lrange
    #         else:
    #             self.range_data[i] = self.range_max
            

    # def ray_casting(self, ray, com_list):
    #     # calculate the minimum distance and collision point between a ray and obstacles
    #     min_lrange = self.range_max
    #     min_int_point = ray[1]
    #     collision_flag = False
        
    #     for com in com_list:
    #         if com.appearance == 'circle': 
    #             flag, int_point, lrange = range_cir_seg(com.center, com.radius, ray)
    #             if flag and lrange < min_lrange:
    #                 min_lrange = lrange
    #                 min_int_point = int_point
    #                 collision_flag = True

    #         if com.appearance == 'rectangle' or com.appearance == 'polygon':
                
    #             edge_list = com.get_edges()
    #             for edge in edge_list:
    #                 flag, int_point, lrange = range_seg_seg(ray, edge)
    #                 if flag and lrange < min_lrange:
    #                     min_lrange = lrange
    #                     min_int_point = int_point
    #                     collision_flag = True
                
    #     return collision_flag, min_int_point, min_lrange

    def plot(self, ax, lidar_color='r', **kwargs):
        
        plot_patch_list = []
        plot_line_list = []

        for start, end in zip(self.global_ray.T, self.global_intersections.T):
            line = ax.plot([start[0], end[0]], [start[1], end[1]], color = lidar_color, alpha=self.alpha, zorder=0)
            plot_line_list.append(line)

        return plot_line_list, plot_patch_list
 
    @staticmethod
    def transform_matrix(x, y, theta):
        trans_matrix = np.array([[x], [y]])
        rot_matrix = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        return trans_matrix, rot_matrix

    @staticmethod
    def distance(point1, point2):
        return sqrt( (point1[0, 0] - point2[0, 0]) ** 2 + (point1[1, 0] - point2[1, 0]) ** 2 )


    @staticmethod
    def relative_position(point1, point2):

        diff = point2 - point1

        dis = round(sqrt( diff[0, 0] ** 2 + diff[1, 0] ** 2 ), 2)
        radian = atan2(diff[1, 0], diff[0, 0])

        return dis, radian

