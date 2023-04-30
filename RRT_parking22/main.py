import math
import random

import matplotlib.pyplot as plt
import numpy as np

from rrt import RRT


def get_random_node_avoid():
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


def get_random_node_start(x,tmp):
    oy=4.3
    rw=3.65
    is_up=random.randint(0,1)
    # is_up=0
    oy+=is_up*rw
    x_rand=random.uniform(0, 10)
    yaw_rand = random.uniform(-math.pi,math.pi)
    # x_rand=x
    # yaw_rand = tmp

    # yaw_rand=85.13108918363861/180*math.pi


    r=abs(random.gauss(0, 1))
    r=0

    l,w,wl=3 ,1.2, 2.2
    theta_head=math.atan2(w/2,l/2+wl/2)
    theta_tail=math.atan2(w/2,l/2-wl/2)
    wlh=l/2+wl/2
    wlt=l/2-wl/2
    lh=math.sqrt(wlh*wlh+w*w/4)
    lt=math.sqrt(wlt*wlt+w*w/4)
    # print(theta_head*180/math.pi,theta_tail*180/math.pi)

    y_limit = 0
    if yaw_rand==math.pi/2:
        if is_up:
            y_limit=wlh
        else:
            y_limit = wlt
    elif yaw_rand==-math.pi/2:
        if not is_up:
            y_limit =wlh
        else:
            y_limit = wlt
    else:
        if is_up:
            if yaw_rand>0 and yaw_rand<math.pi:
                if  yaw_rand < math.pi / 2:
                    y_limit=wlh*math.sin(yaw_rand+theta_head)
                else:
                    y_limit=wlh*math.sin(yaw_rand-theta_head)
            else:
                if yaw_rand < -math.pi / 2:
                    y_limit = wlt * abs(math.sin(yaw_rand + theta_tail))
                else:
                    y_limit = wlt * abs(math.sin(yaw_rand - theta_tail))
        else:
            if not yaw_rand > 0 and yaw_rand < math.pi :
                if yaw_rand < -math.pi / 2:
                    y_limit = wlh * math.sin(yaw_rand + theta_head)
                else:
                    y_limit = wlh * math.sin(yaw_rand - theta_head)
            else:
                if yaw_rand < math.pi / 2:
                    y_limit = wlt * abs(math.sin(yaw_rand + theta_tail))
                else:
                    y_limit = wlt * abs(math.sin(yaw_rand - theta_tail))
        y_limit = abs(y_limit)  # +0.1

    if is_up:
        y_rand=oy-y_limit-r*rw/2
    else:
        y_rand=oy+y_limit+r*rw/2

    print('check',is_up,oy,y_limit,r*rw/2,'----------',y_rand,yaw_rand*180/math.pi)

    return x_rand,y_rand,yaw_rand


def main():
    fig = plt.figure()
    ax = plt.axes()
    plt.plot([0,10],[4.3,4.3],color='black')
    plt.plot([0,10],[4.3+ 3.65,4.3+ 3.65],color='black')
    # plt.plot([-5,5],[0,0],color='black')
    for i in range(100):
        xtmp=0.1*i
        tmp=-math.pi+2*math.pi*i/100
        x,y,theta=get_random_node_start(xtmp,tmp)
        # x,y,theta=get_random_node_end()
        plt.scatter(x,y)
        ax.arrow(x,y, 0.5*math.cos(theta), 0.5*math.sin(theta), length_includes_head=False, head_width=0.05, fc='b', ec='k')
        plt.pause(0.01)
    plt.show()



if __name__ == '__main__':
    main()
