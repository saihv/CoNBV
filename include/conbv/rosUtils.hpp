#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <chrono>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ROSUtils
{
public:
    ROSUtils(int& nDrones_) : nDrones(nDrones_), poseX(nDrones_), readPose(nDrones_, 0)
    {}

    void mapCallback(const PointCloud::ConstPtr& msg)
    {
        if (!mapAvailable) {
            printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
            mapPoints.clear();

            BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
                mapPoints.push_back(cv::Point3f(pt.x, pt.y, pt.z));
                // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

            std::cout << "Read " << mapPoints.size() << " points" << std::endl;
            mapAvailable = true;
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int& idx)
    {
        poseX[idx] = msg->pose.position.x;

        std::cout << "Read pose #" << idx << std::endl;
        readPose[idx] = 1;

        if (std::count(readPose.begin(), readPose.end(), 1) == nDrones)
            posesAvailable = true;

    }

    int nDrones;
    std::vector <cv::Point3f> mapPoints;
    std::vector <int> poseX;
    std::vector <unsigned char> readPose;
    bool mapAvailable, posesAvailable;
};