#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/point_types.h>
#include <string> 
#include <iostream> 
#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

/*#define POINTCLOUD_TYPE pcl::PointXYZRGB

ros::Publisher publisher_joint_4pointcloud;
pcl::PointCloud<POINTCLOUD_TYPE> point_cloud1, point_cloud2, point_cloud3, point_cloud4, point_cloud5, point_cloud6;

void joint_process()
{
    pcl::PointCloud<POINTCLOUD_TYPE> point_cloud_all;

    point_cloud_all += point_cloud1;
    point_cloud_all += point_cloud2;
    point_cloud_all += point_cloud3;
    point_cloud_all += point_cloud4;
    point_cloud_all += point_cloud5;
    point_cloud_all += point_cloud6;

    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>);
    cloud_ptr=point_cloud_all.makeShared();

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_ptr, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    publisher_joint_4pointcloud.publish(cloud_msg);
}

void pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud1);
    joint_process();
}
void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud2);
}
void pointcloud3_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud3);
}
void pointcloud4_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud4);
}
void pointcloud5_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud5);
}
void pointcloud6_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud6);
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "joint_4pointcloud");

    ros::NodeHandle nh_;
    publisher_joint_4pointcloud = nh_.advertise<sensor_msgs::PointCloud2>("joint_4pointcloud", 10);  
    ros::Subscriber pointcloud1_sub = nh_.subscribe("/points_fused1", 10, pointcloud1_callback); 
    ros::Subscriber pointcloud2_sub = nh_.subscribe("/points_fused2", 10, pointcloud2_callback); 
    ros::Subscriber pointcloud3_sub = nh_.subscribe("/points_fused3", 10, pointcloud3_callback); 
    ros::Subscriber pointcloud4_sub = nh_.subscribe("/points_fused4", 10, pointcloud4_callback); 
    ros::Subscriber pointcloud5_sub = nh_.subscribe("/points_fused5", 10, pointcloud5_callback); 
    ros::Subscriber pointcloud6_sub = nh_.subscribe("/points_fused6", 10, pointcloud6_callback); 

    ros::spin(); 
    return 0; 
}*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define POINTCLOUD_TYPE velodyne_pointcloud::PointXYZIR

ros::Publisher publisher_joint_4pointcloud;
pcl::PointCloud<POINTCLOUD_TYPE> point_cloud1, point_cloud2, point_cloud3, point_cloud4, point_cloud5, point_cloud6;

void pointcloud1_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud1);
    

    pcl::PointCloud<POINTCLOUD_TYPE> point_cloud_all;

    point_cloud_all += point_cloud1;
    point_cloud_all += point_cloud2;
    point_cloud_all += point_cloud3;
    point_cloud_all += point_cloud4;
    point_cloud_all += point_cloud5;
    point_cloud_all += point_cloud6;

    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>);
    cloud_ptr=point_cloud_all.makeShared();

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_ptr, cloud_msg);
    cloud_msg.header = laserCloudMsg->header;
    publisher_joint_4pointcloud.publish(cloud_msg);
}
void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud2);
}
void pointcloud3_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud3);
}
void pointcloud4_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud4);
}
void pointcloud5_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud5);
}
void pointcloud6_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{ 
    pcl::fromROSMsg(*laserCloudMsg, point_cloud6);
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "joint_4pointcloud");

    ros::NodeHandle nh_;
    publisher_joint_4pointcloud = nh_.advertise<sensor_msgs::PointCloud2>("joint_4pointcloud", 10);  
    ros::Subscriber pointcloud1_sub = nh_.subscribe("/points_fused1_intensity", 10, pointcloud1_callback); 
    ros::Subscriber pointcloud2_sub = nh_.subscribe("/points_fused2_intensity", 10, pointcloud2_callback); 
    ros::Subscriber pointcloud3_sub = nh_.subscribe("/points_fused3_intensity", 10, pointcloud3_callback); 
    ros::Subscriber pointcloud4_sub = nh_.subscribe("/points_fused4_intensity", 10, pointcloud4_callback); 
    ros::Subscriber pointcloud5_sub = nh_.subscribe("/points_fused5_intensity", 10, pointcloud5_callback); 
    ros::Subscriber pointcloud6_sub = nh_.subscribe("/points_fused6_intensity", 10, pointcloud6_callback); 

    ros::spin(); 
    return 0; 
}
