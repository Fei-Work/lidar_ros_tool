#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

#include <vector>
#include <string>
#include <queue>
#include <cmath>

using namespace std;

ros::Subscriber lL_topic_sub;
ros::Subscriber rL_topic_sub;
ros::Publisher L_topic_pub;
rosbag::Bag record_bag;

vector<float> lL2I_R, rL2I_R, lL2I_t, rL2I_t;
queue<pair<ros::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr>> lL_queue;
queue<pair<ros::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr>> rL_queue;

void transformPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, vector<float> &R, vector<float> &t, pcl::PointCloud<pcl::PointXYZI>::Ptr &out) {
    Eigen::Matrix3f rotation;
    rotation << R[0], R[1], R[2],
                R[3], R[4], R[5],
                R[6], R[7], R[8];
    Eigen::Vector3f translation(t[0], t[1], t[2]);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = rotation;  // 设置旋转矩阵
    transform.translation() = translation;  // 设置平移向量
    pcl::transformPointCloud(*cloud, *out, transform);
}

void lL_topic_Callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    lL_queue.push(make_pair(msg->header.stamp, cloud));
}

void rL_topic_Callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    rL_queue.push(make_pair(msg->header.stamp, cloud));
}


int main(int argc, char** argv){
    ros::init(argc, argv, "merge_lidar");
    ros::NodeHandle nh;

    string lL_topic;
    string rL_topic;
    string L_topic;
    string bag_path;
    bool save_bag;

    nh.param<vector<float>>("lL2I_R",lL2I_R,{1,0,0,0,1,0,0,0,1});
    nh.param<vector<float>>("rL2I_R",rL2I_R,{1,0,0,0,1,0,0,0,1});
    nh.param<vector<float>>("lL2I_t",lL2I_t,{0,0,0});
    nh.param<vector<float>>("rL2I_t",rL2I_t,{0,0,0});
    nh.param<string>("lL_topic",lL_topic,"");
    nh.param<string>("rL_topic",rL_topic,"");
    nh.param<string>("L_topic",L_topic,"/points_raw");
    nh.param<bool>("save_bag", save_bag, true);
    nh.param<string>("bag_path",bag_path,"./data/merged_lidar.bag");


    lL_topic_sub = nh.subscribe(lL_topic, 100, lL_topic_Callback);
    rL_topic_sub = nh.subscribe(rL_topic, 100, rL_topic_Callback);
    L_topic_pub = nh.advertise<sensor_msgs::PointCloud2>(L_topic, 100);

    if(save_bag){
        record_bag.open(bag_path, rosbag::bagmode::Write);
        ROS_INFO("open the record bag:%s", bag_path.c_str());
    }
    
    ros::Rate merge_loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // 合并左右雷达数据
        while (!lL_queue.empty() && !rL_queue.empty()) {
            auto lL_pair = lL_queue.front();
            auto rL_pair = rL_queue.front();

            // 找到时间戳最接近的两个点云
            if (abs((lL_pair.first - rL_pair.first).toSec()) < 0.05) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr lL_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr rL_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);

                // 转换到同一坐标系下
                transformPoint(lL_pair.second, lL2I_R, lL2I_t, lL_cloud_transformed);
                transformPoint(rL_pair.second, rL2I_R, rL2I_t, rL_cloud_transformed);

                // 合并点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                *merged_cloud = *lL_cloud_transformed + *rL_cloud_transformed;

                // 发布合并后的点云
                sensor_msgs::PointCloud2 merged_cloud_msg;
                pcl::toROSMsg(*merged_cloud, merged_cloud_msg);
                merged_cloud_msg.header.stamp = lL_pair.first;
                merged_cloud_msg.header.frame_id = "velodyne";
                L_topic_pub.publish(merged_cloud_msg);

                if(save_bag){
                    record_bag.write(L_topic, lL_pair.first, merged_cloud_msg);
                }

                lL_queue.pop();
                rL_queue.pop();
                ROS_INFO("successful merge:%f", lL_pair.first.toSec());
            } else if (lL_pair.first < rL_pair.first) {
                lL_queue.pop();
                ROS_WARN("fail merge:%f", lL_pair.first.toSec());
            } else {
                rL_queue.pop();
                ROS_WARN("fail merge:%f", lL_pair.first.toSec());
            }
        }

        merge_loop_rate.sleep();
    }
    if(save_bag){
        record_bag.close();
    }
    return 0;
}
