#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_pub", 1);    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);

    sensor_msgs::PointCloud2 output;

    std::string pcd_file_path;
    if (argc>2){
        ROS_WARN("Too many params. Using: pcd_publisher yourpath");
        return -1;
    }
    if (argc<2) {
        ROS_WARN("PCD file path not set. Using: pcd_publisher yourpath");
        return -1;
    }
    if (argc = 2) {
        pcd_file_path = argv[1];
    }

    std::ifstream file(pcd_file_path.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Failed to open PCD file: %s", pcd_file_path.c_str());
        return -1;
    }
    ROS_INFO("Success read the PCD file");
    file.close();
    ROS_INFO("Loading the PCD file ......");
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path, *Cloud);
    pcl::toROSMsg(*Cloud, output);
    output.header.frame_id = "map";
    ROS_INFO("MAP POINTS: %ld", Cloud->size());
    while(ros::ok()){
        pcl_pub.publish(output);
        ros::Duration(2).sleep();
    }
    
    ROS_INFO("Finish");
    
    return 0;
}