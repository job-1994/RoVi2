// PCL Libraries
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace pcl::io;
using namespace cv;

class kalman_filter
{
public:
	kalman_filter();
	~kalman_filter();
	
	static cv::KalmanFilter accelerationKF(PointXYZ initial_point);
	static PointXYZ predict(cv::KalmanFilter &KF);
	static PointXYZ correct(cv::KalmanFilter &KF, cv::Mat measurement);
	static void skipCorrect(cv::KalmanFilter &KF);
	static PointCloud<PointXYZ>::Ptr predictPoints(PointCloud<PointXYZ>::Ptr measures);


	
};