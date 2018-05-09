#include "RoVi2/kalman_filter.hpp"
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include "ur_caros_example/Vision.h"
#include "geometry_msgs/Point.h"

#define DEBUG 1

#define X_BOUND_MIN -0.8698
#define X_BOUND_MAX -0.1376
#define Y_BOUND_MIN 0.1288
#define Y_BOUND_MAX 0.5224
#define Z_BOUND_MIN 0.3920
#define Z_BOUND_MAX 1.4113

#define MIN_POINT_COUNT 13

ros::NodeHandle *nh;
ros::Publisher chatter_pub;
pcl::PointCloud<pcl::PointXYZ> pointCloud;
pcl::PCLPointCloud2 pcl_pc2;
pcl::PointCloud<pcl::PointXYZ> storage_cloud;
sensor_msgs::PointCloud2 pub_msg;
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
cv::Vec4f result_plane;
const int delay_ = 1 * 1000000000;
int iter_ = 0;
int point_count = 0;
int prev_time = 0;
bool got_point = false;
ros::ServiceClient sc;
clock_t time_start = 0;
bool found_point = false;

void cloud_callback(const sensor_msgs::PointCloud2 &cloud_msg)
{
	pcl::fromROSMsg(cloud_msg, *temp_cloud);
	std_msgs::Header header = cloud_msg.header;
	if (header.stamp.nsec - prev_time > delay_ && point_count <= MIN_POINT_COUNT)
	{
		if (DEBUG)
			printf("Unsuccesfull, %i points are not enough\n", point_count);
		iter_ = 0;
		temp_cloud->clear();
		temp_pub_cloud->clear();
		point_count = 0;
	}
	else if (header.stamp.nsec - prev_time > delay_ && point_count >= MIN_POINT_COUNT)
	{
		if (DEBUG)
			printf("Timed out, %i points\n", point_count);
		iter_ = 0;
		temp_cloud->clear();
		temp_pub_cloud->clear();
		point_count = 0;
	}
	if (iter_ == 0)
	{
		time_start = clock();
		prev_time = header.stamp.nsec;
	}
	float x_av = 0;
	float y_av = 0;
	float z_av = 0;
	int size_ = 0;
	for (int i = 0; i < temp_cloud->size(); i++)
	{
		pcl::PointXYZ point = temp_cloud->points[i];
		if (point.z < 2.5)
		{
			x_av = x_av + point.x;
			y_av = y_av + point.y;
			z_av = z_av + point.z;
			size_++;
			got_point = true;
		}
	}
	pcl::PointXYZ point_av;
	point_av.x = x_av / size_;
	point_av.y = y_av / size_;
	point_av.z = z_av / size_;
	cv::Mat row = cv::Mat::ones(1, 4, CV_32F);
	row.at<float>(0, 0) = point_av.x;
	row.at<float>(0, 1) = point_av.y;
	row.at<float>(0, 2) = point_av.z;
	row.at<float>(0, 3) = 1;
	if (!isnan(point_av.x))
	{
		temp_pub_cloud->push_back(point_av);
		pcl::toROSMsg(*temp_pub_cloud, pub_msg);
		pub_msg.header = header;
	}

	if (got_point)
	{
		iter_++;
		point_count++;
		got_point = false;
		if (header.stamp.nsec - prev_time < delay_ && point_count > MIN_POINT_COUNT)
		{
			if (DEBUG)
			{
				// printf("/*-----------------------------------------------------------------------------------*/\n");
				printf("Succesfully gathered throw data, %i points\n", temp_pub_cloud->size());
				// printf("/*-----------------------------------------------------------------------------------*/\n");
			}
			chatter_pub.publish(pub_msg);
			PointCloud<PointXYZ>::Ptr predicted_points = kalman_filter::predictPoints(temp_pub_cloud);
			for (int index = MIN_POINT_COUNT; index < (int)predicted_points->size(); index++)
				printf("x: %f, y: %f, z: %f\n", predicted_points->points[index].x, predicted_points->points[index].y, predicted_points->points[index].z);

			for (int index = MIN_POINT_COUNT; index < (int)predicted_points->size(); index++)
			{
				if (predicted_points->points[index].x < X_BOUND_MAX && predicted_points->points[index].x > X_BOUND_MIN)
					if (predicted_points->points[index].y < Y_BOUND_MAX && predicted_points->points[index].y > Y_BOUND_MIN)
						if (predicted_points->points[index].z < Z_BOUND_MAX && predicted_points->points[index].z > Z_BOUND_MIN)
						{
							printf("point is ok!! x: %f, y: %f, z: %f\n", predicted_points->points[index].x, predicted_points->points[index].y, predicted_points->points[index].z);
							/*CALL SERVICE*/
							found_point = true;
						}
			}

			if(found_point)
			{
											ur_caros_example::Vision newTarget;
							newTarget.request.point.x = -0.4;
							newTarget.request.point.y = -0.1;
							newTarget.request.point.z = 0.35;

							sc.call(newTarget);
				found_point = false;

			}

			// printf("Prediction time %d\n", clock() - time_start);
			// printf("Size of predicted points %i\n", predicted_points->size());
			temp_cloud->clear();
			temp_pub_cloud->clear();
			prev_time = 0;
			iter_ = 0;
			point_count = 0;
		}
		else
		{
			// if(DEBUG) printf("Added point %i to the message, x: %f, y: %f, z: %f, \n", point_count, point.av);
			prev_time = header.stamp.nsec;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_sub");
	nh = new ros::NodeHandle();
	ros::Rate loop_rate(60);
	chatter_pub = nh->advertise<sensor_msgs::PointCloud2>("altered_point", 1000);
	ros::Subscriber sub;
	sub = nh->subscribe("/camera/depth/points", 1, cloud_callback);
	sc = nh->serviceClient<ur_caros_example::Vision>("/vision_coordinates");
	ros::spin();
}
