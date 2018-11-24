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

    }

    void computeProjectionsSingleCamera(int &droneId, std::vector <cv::Point3f> &mapPoints)
    {
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion gq;
        quaternionTFToMsg(q, gq);

        initParams();

        // std::vector <cv::Point2f> projectedPoints;

        int idx = 0;

        cv::Mat tvec, rMat, rvec;

        tvec = (cv::Mat_<float>(3,1) << 0.0, 0.0, 0.0);
        
        tf::Matrix3x3 m(q);
        // quaternionTFToMsg(q2, gq);
        rMat = (cv::Mat_<float>(3,3) << m.getRow(0).getX(), m.getRow(0).getY(), m.getRow(0).getZ(),
                                            m.getRow(1).getX(), m.getRow(1).getY(), m.getRow(1).getZ(),
                                            m.getRow(2).getX(), m.getRow(2).getY(), m.getRow(2).getZ());
        cv::Rodrigues(rMat, rvec);
        // std::cout << "Projecting " << mapPoints.size() << " points" << std::endl;
        projectedPoints.clear();
        cv::projectPoints(mapPoints, rvec, tvec, K, dist, projectedPoints);
        image = cv::Scalar::all(0); 

        std::fill(projections[droneId].begin(), projections[droneId].end(), 0);

        for (int i = 0; i < projectedPoints.size(); ++i) {
            if (projectedPoints[i].x > 0 && projectedPoints[i].x < w && projectedPoints[i].y > 0 && projectedPoints[i].y < h){
                //std::cout << projectedPoints[i].x << "," << projectedPoints[i].y << std::endl;
                image.at<uchar>(std::floor(projectedPoints[i].y), std::floor(projectedPoints[i].x)) = 255;
                projections[droneId][i] = 1;
            }
        }

        //std::cout << "Processed " << projectedPoints.size() << " projections" << std::endl;
        //std::cout << "Found " << std::count(projections[droneId].begin(),projections[droneId].end(),1) << " valid projections" << std::endl;
    }

    void computeProjections(std::vector<cv::Point3f>& mapPoints)
    {
        for (int i = 0; i < nDrones; ++i)
        {
            computeProjectionsSingleCamera(i, mapPoints);
        }
    }

    void computeVisionHeuristics()
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

        heuristics.overlap = numSharedProjections/((double)projections[0].size());
        std::cout << "Overlap is " << heuristics.overlap * 100 << "%" << std::endl;
    }

    std::vector <cv::Point2f> projectedPoints;
    std::vector <std::vector <bool> > projections;
    Heuristics heuristics;

private:
    cv::Mat image;
    cv::Mat K, dist;

    int w, h, binSize;
    float cost;

    int nDrones;

    void initParams()
    {
        w = 640; h = 480;
        binSize = 32;
        image = cv::Mat(h, w, CV_8UC1, cv::Scalar(0));
        K = (cv::Mat_<float>(3,3) << 320, 0, 320, 0, 320, 240, 0, 0, 1);
        dist = (cv::Mat_<float>(5,1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    }
};