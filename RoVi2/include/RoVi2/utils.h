//
// Created by Dirk Kraft on 02/03/2018.
//

#ifndef PLANEFITTINGSOLUTION_UTILS_H
#define PLANEFITTINGSOLUTION_UTILS_H

#include <opencv2/core/core.hpp>

/**
 * Function to save a set of 3D points as colored pointcloud.
 *
 * @param filename - the name of the file the pointcloud should be written to
 * @param points - the points to be saved as a cv::Mat matrix with cv::Vec3f entries
 * @param color - three dimension std::vector of floats containign the red, green and blue values to be used for all
 * points in the cloud. (One color per cloud, not one color per point.)
 */
void savePointsAsPointCloud(std::string filename,
                            cv::Mat points,
                            std::vector<float> color = std::vector<float>(3, 0));

/**
 * Adds a column of ones to the end of the input cv::Mat (CV32F)
 *
 * @param inPoints - cv::Mat that contains one multidimensional float point per row
 * @param outPoints - cv::Mat that contains the same values as the input with dimension (set to one) added to each
 * multidimensional float point
 */
void addOnesDimension(const cv::Mat &inPoints, cv::Mat &outPoints);


/**
 * Fits a plane to the set of 3D points (represented as (X,Y,Z,1)) provided
 * (Solve the equations  of AB=0, where A are the points (X,Y,Z,1) and B are the plane parameters (a,b,c,d))
 *
 * @param points - cv::Mat that has one 4D point (float) per row
 * @param planeParameters - plane parameters as cv::Vec4f
 */
void fitPlaneToPoints(cv::Mat points, cv::Vec4f &planeParameters);


/**
 * Fits a plane to the set of 3D points provided weighted by the weigth matrix
 * (Solve the equation system with the mahalanobis distance (AwAt)B=0, where A is the points (X,Y,Z,1), B is the
 * plane parameters (a,b,c,d) and w is the weight of the points
 * @param points - cv::Mat that has one 4D point (float) per row, N points
 * @param weights - NxN matrix covarince matrix
 * @param planeParameters - plane parameters as cv::Vec4f
 */
void fitPlaneToPointsWeighted(cv::Mat points, cv::Mat weights, cv::Vec4f &planeParameters);

/**
 * Creates #samples 3D points on random locations lying on the plane described by the plane a, b, c, d
 *
 * @return 3D points as cv::Mat
 */
cv::Mat samplePointsOnPlane(float a, float b, float c, float d, int samples);

cv::Mat samplePointsOnPlane(cv::Mat &B, int samples);

void projectPointsToImagePlanes(std::vector<cv::Point3d> points3d,
                                std::vector<cv::Point2d> &imagePointsL,
                                std::vector<cv::Point2d> &imagePointsR, cv::Mat &projectionL,
                                cv::Mat &projectionR);

void setupPlaneParameters(float a, float b, float c, float d, cv::Mat &B);

void copyOver(cv::Mat &plane, std::vector<cv::Point3d> &points3d);

void createNoisyPlane(float sigmaX, float sigmaY,
                      const std::vector<cv::Point2d> &imagePointsL, const std::vector<cv::Point2d> &imagePointsR,
                      const cv::Mat &projR, const cv::Mat &projL,
                      cv::Mat &noisyPlane);

#endif //PLANEFITTINGSOLUTION_UTILS_H
