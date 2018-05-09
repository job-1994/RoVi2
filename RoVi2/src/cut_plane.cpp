#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv2/highgui/highgui.hpp>

    ros::NodeHandle* nh;
   	ros::Publisher chatter_pub;

    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ> storage_cloud;    
    sensor_msgs::PointCloud2 pub_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Mat plane_fitting;    
    cv::Vec4f result_plane;
    int iter = 0;
    float v0 =  0.258386;
    float v1 =  0.015680;
    float v2 =  -0.438867;
    float v3 =  0.860457;

void cloud_callback (const sensor_msgs::PointCloud2& cloud_msg){
    // printf("Reached\n");
    pcl::fromROSMsg(cloud_msg, *temp_cloud);
    std_msgs::Header header = cloud_msg.header;
    // int count = 0;
    float x_av = 0;
    float y_av = 0;
    float z_av = 0;
    int size_ = 0;
    for(int i = 0; i < temp_cloud->size(); i++)
    {
        pcl::PointXYZ point = temp_cloud->points[i];
        // if(point.z < ((-v0 * point.x - v1 * point.y - v3) / v2) && point.y < 0 && point.z < 2 && point.z > 0.5)
        if(point.z < 2.5)
        {
            x_av = x_av + point.x;
            y_av = y_av + point.y;
            z_av = z_av + point.z;
            size_++;
            // printf("point y %f\n", point.y);
            // temp_pub_cloud->push_back(point);
            // count++;
            // printf("Point x: %f, Point y: %f, Point Z: %f\n", point.x, point.y, point.z);
            temp_pub_cloud->push_back(point); 
        }
    }   
    pcl::PointXYZ point_av;
    point_av.x = x_av/size_;
    point_av.y = y_av/size_;
    point_av.z = z_av/size_;
    
    // temp_pub_cloud->push_back(point_av); 
    pcl::toROSMsg(*temp_pub_cloud, pub_msg);
    // pub_msg.header.frame_id = "camera_depth_optical_frame";
    pub_msg.header = header;
    // printf()
    chatter_pub.publish(pub_msg);
    temp_cloud->clear();
    temp_pub_cloud->clear();
    // printf("plane_fitting rows: %i\n", plane_fitting.rows);

}



int main (int argc, char** argv) 
{
     ros::init (argc, argv, "cloud_sub");
     nh = new ros::NodeHandle();
     ros::Rate loop_rate(60);
     chatter_pub = nh->advertise<sensor_msgs::PointCloud2>("cut_projection", 1000);
     ros::Subscriber sub;
     sub = nh->subscribe("/camera/depth/points", 1, cloud_callback);
     ros::spin();
}

