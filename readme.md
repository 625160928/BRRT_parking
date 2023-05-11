# 基于Bi-RRT*的泊车采样优化

## 前言
本代码是基于python robotic的rrt代码的基础上一路改过来的

还有，python robotic的rrt*+reed shepp代码bug一堆

rrt代码层级关系 RRT-》RRT*-》RRT*+reed shepp-》BRRT*+reed shepp(我的)

## 代码简单介绍
### RRT*
简单介绍下rrt*，就是在rrt的基础上让所有节点尽可能的减少弯弯绕绕的路径。

每个节点与他的父亲，父亲的父亲，父亲的父亲的父亲……如果存在直接连接的路径那就直接连接

### RRT+reed shepp
RRT连接采样点使用的是直线，在节点连出的几根线之间是折角的样子，但是车辆无法跟踪折角的路线

所以使用符合车辆动力学的方法连接采样点之间的路线，也就是reed shepp曲线

reed shepp相比dubins曲线，reed shepp可以支持倒车的规划

毕竟这个项目是泊车的项目，泊车的项目车没法规划出倒车的路线是否太过……精彩？

### Bi-RRT
有一个一直没搞懂，Bi-RRT好像有个名字叫RRT-connect？好像还有个简称叫BRRT?

在rrt的基础上构建两个搜索树，一个起点搜索树一个终点搜索树，两个树平衡生长直到互相连接




## 怎么运行
```
cd RRT_parking
python ir_sim.py
```
实际上我是在pycharm运行的，将hybrid_astar和ir_sim两个文件夹设置为source root才能正常运行（这应该是小问题吧？）

#运行效果