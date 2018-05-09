//
// Created by Dirk Kraft on 02/03/2018.
//

#include "utils.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>


void savePointsAsPointCloud(std::string filename,
                            cv::Mat points,
                            std::vector<float> color) {
    pcl::PointXYZRGB p_default;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst(new pcl::PointCloud<pcl::PointXYZRGB>(points.rows, 1, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        cv::Vec3f p = points.at<cv::Vec3f>(i);
        pcl::PointXYZRGB pn;
        pn.x = p[0];
        pn.y = p[1];
        pn.z = p[2];
        pn.r = color[0];
        pn.g = color[1];
        pn.b = color[2];
        dst->points.at(i) = pn;
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}

void addOnesDimension(const cv::Mat &inPoints, cv::Mat &outPoints) {
    cv::Mat tempPoints;
    cv::transpose(inPoints, tempPoints);
    cv::Mat row = cv::Mat::ones(1, inPoints.rows, CV_32F);
    tempPoints.push_back(row);
    cv::transpose(tempPoints, outPoints);
}

void fitPlaneToPoints(cv::Mat points, cv::Vec4f &planeParameters) {

    cv::Mat w, u, vt;
    cv::SVD::compute(points, w, u, vt, cv::DECOMP_SVD);
    //Take out the last column of vt
    planeParameters = vt.at<cv::Vec4f>(vt.rows - 1);
}

void fitPlaneToPointsWeighted(cv::Mat points, cv::Mat weights, cv::Vec4f &planeParameters) {
    cv::Mat AT;
    cv::transpose(points, AT);
    cv::Mat w, u, vt;
    cv::SVD::compute(AT * weights * points, w, u, vt, cv::DECOMP_SVD);
    //Take out the last column of vt
    planeParameters = vt.at<cv::Vec4f>(vt.rows - 1);
}

cv::Mat samplePointsOnPlane(float a, float b, float c, float d, int samples) {
    cv::Mat planePoints = cv::Mat(samples, 3, CV_32F);
    for (int i = 0; i < samples; i++) {
        float X, Y, Z;
        //Pick random points of X and Y and compute Z
        X = 1.5 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 - 1);
        Y = 1.5 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 - 1);
        Z = (-a * X - b * Y - d) / c;
        planePoints.at<float>(i, 0) = X;
        planePoints.at<float>(i, 1) = Y;
        planePoints.at<float>(i, 2) = Z;
    }
    return planePoints;
}

cv::Mat samplePointsOnPlane(cv::Mat &B, int samples) {
    float a, b, c, d;
    a = B.at<float>(0);
    b = B.at<float>(1);
    c = B.at<float>(2);
    d = B.at<float>(3);
    return samplePointsOnPlane(a, b, c, d, samples);
}

void projectPointsToImagePlanes(std::vector<cv::Point3d> points3d,
                                std::vector<cv::Point2d> &imagePointsL,
                                std::vector<cv::Point2d> &imagePointsR, cv::Mat &projectionL,
                                cv::Mat &projectionR) {
    // Create pseudo projection matrix
    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrinsic matrix
    intrisicMat.at<double>(0, 0) = 1.6415318549788924e+003;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;

    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 1.7067753507885654e+003;
    intrisicMat.at<double>(2, 1) = 0;

    intrisicMat.at<double>(0, 2) = 5.3262822453148601e+002;
    intrisicMat.at<double>(1, 2) = 3.8095355839052968e+002;
    intrisicMat.at<double>(2, 2) = 1;

    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64F); // Rotation vector

    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = -1;
    tVec.at<double>(1) = 1;
    tVec.at<double>(2) = -9;

    cv::Mat tVec2(3, 1, cv::DataType<double>::type); // Translation vector
    tVec2.at<double>(0) = 1;
    tVec2.at<double>(1) = 1;
    tVec2.at<double>(2) = -9;

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // Distortion vector

    std::vector<cv::Point2d> projectedPoints;

    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    transform.at<double>(0, 3) = -1;
    transform.at<double>(1, 3) = 1;
    transform.at<double>(2, 3) = -9;

    cv::Mat tmp = intrisicMat;
    cv::Mat tmp1 = cv::Mat::zeros(3, 1, CV_64F);
    hconcat(tmp, tmp1, tmp);
    projectionL = tmp * transform;

    // change external parameter for right camera
    transform.at<double>(0, 3) = 1;
    projectionR = tmp * transform;

    // Project to left camera
    cv::projectPoints(points3d, rVec, tVec, intrisicMat, distCoeffs,
                      imagePointsL);

    cv::Mat ML(760, 1000, CV_8UC3, cv::Scalar(0, 0, 0)), MR(760, 1065, CV_8UC3, cv::Scalar(0, 0, 0));
    for (unsigned int i = 0; i < imagePointsL.size(); ++i) {
        if (imagePointsL[i].x < 1065 && imagePointsL[i].x > 0) {
            if (imagePointsL[i].y < 760 && imagePointsL[i].y > 0) {
                ML.at<float>(round(imagePointsL[i].y), round(imagePointsL[i].x), 0) = 125;
                // Add neighbouring points to enable visualisation on projected image
                ML.at<float>(round(imagePointsL[i].y) + 1, round(imagePointsL[i].x), 0) = 125;
                ML.at<float>(round(imagePointsL[i].y) - 1, round(imagePointsL[i].x), 0) = 125;
                ML.at<float>(round(imagePointsL[i].y), round(imagePointsL[i].x) - 1, 0) = 125;
                ML.at<float>(round(imagePointsL[i].y), round(imagePointsL[i].x) + 1, 0) = 125;
            }
        }
    }
    //Save projected image to file
    cv::imwrite("ProjectedLeft.png", ML);

    // Project to right camera
    cv::projectPoints(points3d, rVec, tVec2, intrisicMat, distCoeffs,
                      imagePointsR);

    for (unsigned int i = 0; i < imagePointsR.size(); ++i) {
        if (imagePointsR[i].x < 1000 && imagePointsR[i].x > 0) {
            if (imagePointsR[i].y < 760 && imagePointsR[i].y > 0) {
                MR.at<float>(round(imagePointsR[i].y), round(imagePointsR[i].x), 0) = 125;
                MR.at<float>(round(imagePointsR[i].y) + 1, round(imagePointsR[i].x), 0) = 125;
                MR.at<float>(round(imagePointsR[i].y) - 1, round(imagePointsR[i].x), 0) = 125;
                MR.at<float>(round(imagePointsR[i].y), round(imagePointsR[i].x) - 1, 0) = 125;
                MR.at<float>(round(imagePointsR[i].y), round(imagePointsR[i].x) + 1, 0) = 125;
            }
        }
    }
    //Save projected image to file
    cv::imwrite("ProjectedRight.png", MR);
}

void setupPlaneParameters(float a, float b, float c, float d, cv::Mat &B) {
    B.at<float>(0) = a;
    B.at<float>(1) = b;
    B.at<float>(2) = c;
    B.at<float>(3) = d;
}

void copyOver(cv::Mat &plane, std::vector<cv::Point3d> &points3d) {
    points3d.clear();
    for (unsigned int i = 0; i < plane.rows; i++) {
        points3d.push_back(cv::Point3d(plane.at<float>(i, 0), plane.at<float>(i, 1), plane.at<float>(i, 2)));
    }
}

void createNoisyPlane(float sigmaX, float sigmaY,
                      const std::vector<cv::Point2d> &imagePointsL, const std::vector<cv::Point2d> &imagePointsR,
                      const cv::Mat &projR, const cv::Mat &projL,
                      cv::Mat &noisyPlane) {
    for (unsigned int i = 0; i < imagePointsL.size(); i++) {
        cv::Mat pnts3D(1, 1, CV_64FC4);
        cv::Mat cam0pnts(1, 1, CV_64FC2);
        cv::Mat cam1pnts(1, 1, CV_64FC2);
        cam0pnts.at<cv::Vec2d>(0)[0] =
                imagePointsL.at(i).x + (2 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 1) * sigmaX;
        cam0pnts.at<cv::Vec2d>(0)[1] =
                imagePointsL.at(i).y + (2 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 1) * sigmaY;
        cam1pnts.at<cv::Vec2d>(0)[0] =
                imagePointsR.at(i).x + (2 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 1) * sigmaX;
        cam1pnts.at<cv::Vec2d>(0)[1] =
                imagePointsR.at(i).y + (2 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 1) * sigmaY;

        cv::triangulatePoints(projL(cv::Rect(0, 0, 4, 3)), projR(cv::Rect(0, 0, 4, 3)), cam0pnts, cam1pnts, pnts3D);

        // Normalize the homogenous coordinates
        cv::Mat pnts3DNorm = pnts3D / pnts3D.at<double>(3, 0);
        noisyPlane.at<float>(i, 0) = pnts3DNorm.at<double>(0, 0);
        noisyPlane.at<float>(i, 1) = pnts3DNorm.at<double>(1, 0);
        noisyPlane.at<float>(i, 2) = pnts3DNorm.at<double>(2, 0);
    }
}