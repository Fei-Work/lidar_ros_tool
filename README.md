# lidar_tool

使用Veloyne格式的lidar数据所使用的工具包包括：点云地图发布可视化，点云体素滤波，不同话题的lidar数据合并。

## Build

```
cd src
git clone https://github.com/Fei-Work/lidar_ros_tool.git
cd ..
catkin build
```
对于不同的包有不同的配置文件

## 点云地图发布可视化

```
cd /your_pcd_path
rosrun lidar_tool map_show your_pcd
```
## 点云滤波
```
roslaunch lidar_tool cloud_fliter.launch
```

## 不同话题的lidar数据合并
这里主要使用的是KAIST数据集中的left和right的veloyne16雷达bag数据，将左右雷达数据转化到body坐标系下，并拼接为一个雷达话题发布.

*关于KAIST数据集如何转化为bag形式的数据，参看：[kaist2bag](https://github.com/tsyxyz/kaist2bag)*

**EXAMPLE**

```
roslaunch lidar_tool merge_lidar_KAIST.launch
rosbag play yourbag
```