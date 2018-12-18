#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <cv_bridge/cv_bridge.h>

struct Heuristics
{
    double visibility;
    double overlap;
    double span;
    double bzratio;
    double vergence;
};

class VisionUtils
{
public:
    VisionUtils(int &numDrones_, std::vector<cv::Point3f>& mapPoints_) : nDrones(numDrones_), projections(numDrones_, std::vector <bool>(mapPoints_.size()))
    {
        initParams();
    }

    void computeProjectionsSingleCamera(int &droneId, std::vector <double>& state, std::vector <cv::Point3f> &mapPoints, std::vector <cv::Point2f>& projectedPoints)
    {
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        //cv::Mat tvec, rMat, rvec;

        //tvec = (cv::Mat_<float>(3,1) << state[0], state[1], state[2]);

        Eigen::Matrix3d rMat;
        Eigen::Vector3d tvec;
        Eigen::MatrixXd P(3, 4);

        tvec << -state[0], -state[1], -state[2];
        
        tf::Matrix3x3 m(q);
        rMat << m.getRow(0).getX(), m.getRow(0).getY(), m.getRow(0).getZ(),
                                            m.getRow(1).getX(), m.getRow(1).getY(), m.getRow(1).getZ(),
                                            m.getRow(2).getX(), m.getRow(2).getY(), m.getRow(2).getZ();
        //cv::Rodrigues(rMat, rvec);
        projectedPoints.clear();
        //cv::projectPoints(mapPoints, rvec, tvec, K, dist, projectedPoints);

        P << rMat, tvec;
        projectPoints(mapPoints, projectedPoints, K, P);

        image = cv::Scalar::all(0); 
        std::fill(projections[droneId].begin(), projections[droneId].end(), 0);

        for (int i = 0; i < projectedPoints.size(); ++i) {
            if (projectedPoints[i].x > 0 && projectedPoints[i].x < w && projectedPoints[i].y > 0 && projectedPoints[i].y < h){
                //std::cout << projectedPoints[i].x << "," << projectedPoints[i].y << std::endl;
                image.at<uchar>(std::floor(projectedPoints[i].y), std::floor(projectedPoints[i].x)) = 255;
                projections[droneId][i] = 1;
            }
        }

        std::string filename = "/home/sai/proj_" + std::to_string(droneId) + ".png";
        cv::imwrite(filename, image); 
        /*
        
        std::cout << "Processed " << projectedPoints.size() << " projections" << std::endl;
        std::cout << "Found " << std::count(projections[droneId].begin(),projections[droneId].end(),1) << " valid projections" << std::endl;
        */
    }

    void computeProjections(const double *x, int dim, std::vector<cv::Point3f>& mapPoints)
    {
        std::vector <double> state(4);
        int dimIdx = 0;
        
        int visibility = 0;
        int span = 0;

        for (int i = 0; i < nDrones; ++i)
        {
            state[0] = x[dimIdx];
            state[1] = x[dimIdx + 1];
            state[3] = x[dimIdx + 2];
            state[4] = x[dimIdx + 3];

            std::cout << "Evaluating pose: (" << state[0] << "," << state[1] << "," << state[2] << ")" << std::endl;
        
            std::vector <cv::Point2f> projectedPoints;
            computeProjectionsSingleCamera(i, state, mapPoints, projectedPoints);

            visibility += ((double)projectedPoints.size()/(double)mapPoints.size());

            std::cout << "Visibility: " << visibility * 100 << std::endl;

            if(projectedPoints.size() == 0)
                span += 100;
            else
                span += computeSpan();

            dimIdx += 4;
        }
        heuristics.overlap = computeOverlap();
        heuristics.visibility = (double)visibility/(double)nDrones;
        heuristics.span = (double)span/(double)nDrones;
    }

    double computeOverlap()
    {
        std::vector <bool> sharedProjections(projections[0].size());

        for (int i = 0; i < nDrones - 1; ++i) {
            if (i == 0) {
                for (int projIdx = 0; projIdx < projections[i].size(); ++projIdx) {
                    sharedProjections[projIdx] = projections[i][projIdx] && projections[i+1][projIdx];
                }
            }
            else {
                for (int projIdx = 0; projIdx < projections[i+1].size(); ++projIdx)
                    sharedProjections[projIdx] = sharedProjections[projIdx] && projections[i+1][projIdx];
            }
        }

        double numSharedProjections = std::count (sharedProjections.begin(), sharedProjections.end(), 1);

        return numSharedProjections/((double)projections[0].size());
        //std::cout << "Overlap is " << heuristics.overlap * 100 << "%" << std::endl;
    }

    double computeSpan()
    {        
        int *binPixCount = new int[numBins];
        double binPixAcc = 0.0;
        int binIdx = 0;
        cv::Rect roi;

        for (int y = 0; y < image.cols-binSize; y += binSize) {
            for (int x = 0; x < image.rows-binSize; x += binSize) {
                roi = cv::Rect(y, x, binSize, binSize);    
                binPixCount[binIdx] = cv::countNonZero(image(roi));
                
                binPixAcc += binPixCount[binIdx];
                binIdx++;
            }
        }

        double mean = binPixAcc / (double)numBins;
        double varAcc = 0.0;
        for (int i = 0; i < numBins; ++i) 
            varAcc += (binPixCount[i] - mean) * (binPixCount[i] - mean);

        double variance = varAcc / numBins;
        // free(binPixCount);
        delete[] binPixCount;
        return variance;
    }

    // std::vector <cv::Point2f> projectedPoints;
    std::vector <std::vector <bool> > projections;
    Heuristics heuristics;

private:
    cv::Mat image;
    //cv::Mat K, dist;
    Eigen::Matrix3d K;

    int w, h, binSize, numBins;
    float cost;

    int nDrones;

    void initParams()
    {
        w = 640; h = 480;
        binSize = 32;
        image = cv::Mat(h, w, CV_8UC1, cv::Scalar(0));
        //K = (cv::Mat_<float>(3,3) << 320, 0, 320, 0, 320, 240, 0, 0, 1);
        K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
        //dist = (cv::Mat_<float>(5,1) << 0.0, 0.0, 0.0, 0.0, 0.0);
        numBins = (w/binSize - 1) * (h/binSize - 1);
    }

    void projectPoints(std::vector<cv::Point3f>& mapPoints, std::vector<cv::Point2f>& imagePoints, Eigen::Matrix3d& K, Eigen::MatrixXd& P)
    {
        for (int i = 0; i < mapPoints.size(); ++i) {
            Eigen::Vector4d objectPt;
            objectPt << mapPoints[i].x, mapPoints[i].y, mapPoints[i].z , 1;

            Eigen::Vector3d projection;
            projection = K * P * objectPt;

            // std::cout << "(" << proj(0,0)/proj(2,0) << "," << proj(1,0)/proj(2,0) << "," << proj(2,0) << ")" << std::endl;

            if (isValid(projection)) {
                imagePoints.push_back(cv::Point2f(projection(0,0)/projection(2,0), projection(1,0)/projection(2,0)));
            }
        }
    }

    bool isValid(Eigen::Vector3d& projection)
    {
        if (projection(2,0) > 0) {
            if (projection(0,0)/projection(2,0) > 0 && projection(0,0)/projection(2,0) < w && projection(1,0)/projection(2,0) > 0 && projection(1,0)/projection(2,0) < h)
                return true;
        }
        else
            return false;
    }
};