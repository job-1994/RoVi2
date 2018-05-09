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
cv::Vec4f result_plane;
int iter = 0;


void cloud_callback (const sensor_msgs::PointCloud2& cloud_msg){
	pcl::fromROSMsg(cloud_msg, *temp_cloud);
	std_msgs::Header header = cloud_msg.header;
	float x_av = 0;
	float y_av = 0;
	float z_av = 0;
	int size_ = 0;
	for(int i = 0; i < temp_cloud->size(); i++)
	{
		pcl::PointXYZ point = temp_cloud->points[i];
		if(point.z < 2.5)
		{
			x_av = x_av + point.x;
			y_av = y_av + point.y;
			z_av = z_av + point.z;
			size_++;
			// temp_pub_cloud->push_back(point);
		}
	}   
	printf("Points in plane %i\n", size_);
	pcl::PointXYZ point_av;
	point_av.x = x_av/size_;
	point_av.y = y_av/size_;
	point_av.z = z_av/size_;
	cv::Mat row = cv::Mat::ones(1, 4, CV_32F);
	row.at<float>(0,0) = point_av.x;
	row.at<float>(0,1) = point_av.y;
	row.at<float>(0,2) = point_av.z;
	row.at<float>(0,3) = 1;
	
	temp_pub_cloud->push_back(point_av); 
	pcl::toROSMsg(*temp_pub_cloud, pub_msg);
	pub_msg.header = header;
	chatter_pub.publish(pub_msg);
	temp_cloud->clear();
	temp_pub_cloud->clear();
	iter++;
}



int main (int argc, char** argv) 
{
	 ros::init (argc, argv, "cloud_sub");
	 nh = new ros::NodeHandle();
	 ros::Rate loop_rate(60);
	 chatter_pub = nh->advertise<sensor_msgs::PointCloud2>("altered_point", 1000);
	 ros::Subscriber sub;
	 sub = nh->subscribe("/camera/depth/points", 1, cloud_callback);
	 ros::spin();
}

