#include "RoVi2/kalman_filter.hpp"


#define DT 0.25		// timestep


#define PROCESS_NOISE 0.1
#define MEASUREMENT_NOISE 0.05 // Smaller noise, means the measures are more important
#define ERROR_COV_POST_NOISE 0.1

#define POINTS_USED 100


// Namespaces
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace pcl::io;
using namespace cv;



// Method for converting the point in the camera frame to base frame
PointXYZ kalman_filter::converting_cam_to_base(PointXYZ camera_point){
        // Initialize the trasnformation matrix
        Mat transform_matrix = Mat::zeros(4, 4, CV_32F);

        // First Line
        transform_matrix.at<float>(0, 0) = 0.4434;
        transform_matrix.at<float>(0, 1) = 0.0292;
        transform_matrix.at<float>(0, 2) = -0.8959;
        transform_matrix.at<float>(0, 3) = 0.6216;

        // Second Line
        transform_matrix.at<float>(1, 0) = 0.8958;
        transform_matrix.at<float>(1, 1) = 0.0215;
        transform_matrix.at<float>(1, 2) = 0.4440;
        transform_matrix.at<float>(1, 3) = 0.1938;

        // Third Line
        transform_matrix.at<float>(2, 0) = 0.0322;
        transform_matrix.at<float>(2, 1) = -0.9993;
        transform_matrix.at<float>(2, 2) = -0.0166;
        transform_matrix.at<float>(2, 3) = 0.6566;

        // Fourth Line
        transform_matrix.at<float>(3, 0) = 0;
        transform_matrix.at<float>(3, 1) = 0;
        transform_matrix.at<float>(3, 2) = 0;
        transform_matrix.at<float>(3, 3) = 1;

        Mat homogeneous_cam_point = Mat::zeros(4, 1, CV_32F);
        homogeneous_cam_point.at<float>(0, 0) = camera_point.x;
        homogeneous_cam_point.at<float>(1, 0) = camera_point.y;
        homogeneous_cam_point.at<float>(2, 0) = camera_point.z;
        homogeneous_cam_point.at<float>(3, 0) = 1;

        Mat homogeneous_robot_point = Mat::zeros(4, 1, CV_32F);
        homogeneous_robot_point = transform_matrix*homogeneous_cam_point;

        PointXYZ base_point(homogeneous_robot_point.at<float>(0, 0),
                                                homogeneous_robot_point.at<float>(1, 0),
                                                homogeneous_robot_point.at<float>(2, 0));
        return (base_point);
}



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
