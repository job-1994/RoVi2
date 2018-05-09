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
cv::Mat plane_points;    
cv::Vec4f result_plane;
int iter = 0;


void samplePointsOnPlane(float a, float b, float c, float d, int samples) {
	pcl::PointXYZ point_plane;
	for (int i = 0; i < samples; i++) {
		float X, Y, Z;
		//Pick random points of X and Y and compute Z
		X = 1.5 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 - 1);
		Y = 1.5 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 - 1);
		Z = (-a * X - b * Y - d) / c;
		point_plane.x = X;
		point_plane.y = Y;
		point_plane.z = Z;
		// printf("x: %f, y: %f, z: %f\n", X, Y, Z);
		temp_pub_cloud->push_back(point_plane); 
	}
    // printf("header frame id: %c\n", pub_msg.header.frame_id);
	pcl::toROSMsg(*temp_pub_cloud, pub_msg);
    pub_msg.header.frame_id = "camera_depth_optical_frame";
	chatter_pub.publish(pub_msg);
	temp_pub_cloud->clear();
}


int main (int argc, char** argv) 
{
	 ros::init (argc, argv, "cloud_sub");
	 nh = new ros::NodeHandle();
	 ros::Rate loop_rate(60);
	 chatter_pub = nh->advertise<sensor_msgs::PointCloud2>("plane", 1000);
	 while(ros::ok())
	 	samplePointsOnPlane(0.258386, 0.015680, -0.438867, 0.860457, 500);
	 ros::spin();
}

