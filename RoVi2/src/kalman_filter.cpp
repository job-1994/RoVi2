#include "RoVi2/kalman_filter.hpp"


#define DT 0.25		// timestep


#define PROCESS_NOISE 0.1
#define MEASUREMENT_NOISE 0.05 // Smaller noise, means the measures are more important
#define ERROR_COV_POST_NOISE 0.1

#define POINTS_USED 30


// Namespaces
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace pcl::io;
using namespace cv;


cv::KalmanFilter kalman_filter::accelerationKF(PointXYZ initial_point)
{
		cv::KalmanFilter KF(9, 3, 0);

		// Set the parameters: DT*DT should be 0.5*DT*DT but I get better results
		// without the 0.5

		KF.transitionMatrix = (cv::Mat_<float>(9, 9) << 1, 0, 0, DT, 0, 0, DT*DT, 0, 0,
														0, 1, 0, 0, DT, 0, 0, DT*DT, 0,
														0, 0, 1, 0, 0, DT, 0, 0, DT*DT,
														0, 0, 0, 1, 0, 0, DT, 0, 0,
														0, 0, 0, 0, 1, 0, 0, DT, 0,
														0, 0, 0, 0, 0, 1, 0, 0, DT,
														0, 0, 0, 0, 0, 0, 1, 0, 0,
														0, 0, 0, 0, 0, 0, 0, 1, 0,
														0, 0, 0, 0, 0, 0, 0, 0, 1);

		// statePost is the corrected state
		KF.statePost.at<float>(0) = initial_point.x;
		KF.statePost.at<float>(1) = initial_point.y;
		KF.statePost.at<float>(2) = initial_point.z;

		// setIdentity() initializes a scaled identity matrix
		setIdentity(KF.measurementMatrix);
		// The second argument replaces the ones by the respective number
		setIdentity(KF.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

		return KF;
}

PointXYZ kalman_filter::predict(cv::KalmanFilter &KF)
{
		// Do a prediction and return as cv::Point
		cv::Mat pred = KF.predict();
		PointXYZ predicted_point;
		// The coordinates of the point are integers, because they have to be plotted
		// at pixel locations
		predicted_point.x = pred.at<float>(0);
		predicted_point.y = pred.at<float>(1);
		predicted_point.z = pred.at<float>(2);

		return predicted_point;
}

PointXYZ kalman_filter::correct(cv::KalmanFilter &KF, cv::Mat measurement)
{
		// Do a correction and return as cv::Point
		cv::Mat corr = KF.correct(measurement);
		PointXYZ corrected_point;
		corrected_point.x = corr.at<float>(0);
		corrected_point.y = corr.at<float>(1);
		corrected_point.z = corr.at<float>(2);

		return corrected_point;
}

void kalman_filter::skipCorrect(cv::KalmanFilter &KF)
{
		// Skip correction as described in pdf
		KF.statePre.copyTo(KF.statePost);
		KF.errorCovPre.copyTo(KF.errorCovPost);
}




PointCloud<PointXYZ>::Ptr kalman_filter::predictPoints(PointCloud<PointXYZ>::Ptr measures)
{
	PointXYZ initial_point = measures->points[0];
	cv::KalmanFilter KF = accelerationKF(initial_point);

	PointCloud<PointXYZ>::Ptr predictions(new PointCloud<PointXYZ>);
	// Not sure if we should have '<' or '<='
	for (size_t point_index = 0; point_index <= POINTS_USED; point_index++) 
	{
		// Predicted
		PointXYZ kf_predicted_point = predict(KF);

		// Measurements values: Kalman Filter works with cv::Mat
		cv::Mat_<float> meas(3, 1);
		meas(0) = measures->points[point_index].x;
		meas(1) = measures->points[point_index].y;
		meas(2) = measures->points[point_index].z;

		PointXYZ measured_point = measures->points[point_index];

		// Correct for all the points, except the last five
		if (point_index < measures->size())
			correct(KF, meas);
		else 
			skipCorrect(KF);	

		predictions->push_back(kf_predicted_point);

	}
	return predictions;
}
