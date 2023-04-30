import math
import random

import matplotlib.pyplot as plt
import numpy as np

from rrt import RRT


def get_random_node_start():
    rw = 3.65
    cr = rw / 5 / 2
    pd = random.randint(0, 1)
    oy = pd * rw + 4.3
    ox = random.uniform(0, 10)
    r = random.gauss(4 * cr, cr * cr)
    theta = random.uniform(0, math.pi)
    if pd:
        theta = -theta
    x_rand = ox + r * math.cos(theta)
    y_rand = oy + r * math.sin(theta)
    if random.randint(0, 1):
        yaw_rand = math.atan2(ox - x_rand,y_rand - oy)
    else:
        yaw_rand = math.atan2( ox - x_rand,y_rand - oy) + math.pi

    print(ox, oy, theta, r, '-----', x_rand, y_rand, yaw_rand)



    return x_rand, y_rand, yaw_rand

def get_random_node_end(goal_rate=None):

        oy = 0
        ox = 0
        oyaw=math.pi/2

        r = 5*random.uniform(0, 1)

        theta =oyaw+ random.gauss(0,0.5)*math.pi/4

        if random.randint(0, 1):
            theta = theta+math.pi

        x_rand = ox + r * math.cos(theta)
        y_rand = oy + r * math.sin(theta)
        yaw_rand=oyaw+math.pi/2*random.gauss(0,1)



        return x_rand, y_rand, yaw_rand

def main():
    fig = plt.figure()
    ax = plt.axes()
    # plt.plot([0,10],[4.3,4.3],color='black')
    # plt.plot([0,10],[4.3+ 3.65,4.3+ 3.65],color='black')
    plt.plot([-5,5],[0,0],color='black')
    for i in range(1000):
        x,y,theta=get_random_node_end()
        print(x,y,theta*180/math.pi)
        plt.scatter(x,y)
        ax.arrow(x,y, 0.5*math.cos(theta), 0.5*math.sin(theta), length_includes_head=False, head_width=0.05, fc='b', ec='k')
        plt.pause(0.01)




if __name__ == '__main__':
    main()
