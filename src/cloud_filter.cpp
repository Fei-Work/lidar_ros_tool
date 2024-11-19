#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "cloud_filter");
    ros::NodeHandle nh;
    std::string pcd_path;
    std::string save_path;
    double grid_size;

    nh.param<std::string>("cloud_path", pcd_path, "");
    nh.param<double>("grid_size", grid_size, 1.0);

    size_t pos = pcd_path.rfind(".pcd");
    if (pos != std::string::npos) {
        std::string base_name = pcd_path.substr(0, pos);  // 获取不带扩展名的部分
        save_path = base_name + "_filtered" + ".pcd";  // 拼接新的文件名
    } 
    else {
        ROS_ERROR("Invalid pcd file path.");
        return -1;
    }

    ROS_INFO("cloud_path: %s", pcd_path.c_str());
    ROS_INFO("save_path: %s", save_path.c_str());
    ROS_INFO("grid_size: %f", grid_size);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
-
    pcl::io::loadPCDFile(pcd_path, *cloud);
    ROS_INFO("cloud points: %ld m", cloud->size());

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(grid_size, grid_size, grid_size); 
    voxel_grid.filter(*cloud_filtered);

    pcl::visualization::PCLVisualizer viewer("after filter view");
    viewer.addPointCloud<pcl::PointXYZI>(cloud_filtered, "sample cloud");

    while(!viewer.wasStopped()){
        viewer.spinOnce(100);
    }
    pcl::io::savePCDFile(save_path, *cloud_filtered);

    return 0;
}