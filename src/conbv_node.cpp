#include "libcmaes/cmaes.h"
#include <iostream>
#include "conbv/rosUtils.hpp"
#include "conbv/Optimizer.hpp"
#include "conbv/visionUtils.hpp"
#include "ros/ros.h"

using namespace libcmaes;

FitFunc fsphere = [](const double *x, const int N)
{
    double val = 0.0;
    for (int i=0;i<N;i++)
        val += x[i]*x[i];
    return val;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "conbv_node");
  	ros::NodeHandle nh;

	int nDrones = 2;

  	ROSUtils rosUtils(nDrones);
	Optimizer optimizer;

  	ros::Subscriber sub = nh.subscribe<PointCloud>("map", 1, &ROSUtils::mapCallback, &rosUtils);

	for (int droneIdx = 0; droneIdx < nDrones; droneIdx++) {
		std::string topicName = "coloc/drone" + std::to_string(droneIdx) + "/pose";
		ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(topicName, 1, boost::bind(&ROSUtils::poseCallback, &rosUtils, _1, droneIdx));
	}

  	rosUtils.mapAvailable = false;
	rosUtils.posesAvailable = false;

  	while(!rosUtils.mapAvailable || !rosUtils.posesAvailable && ros::ok())
    	ros::spinOnce();

	VisionUtils visionUtils(nDrones, rosUtils.mapPoints);
	visionUtils.computeProjections(rosUtils.mapPoints);
	visionUtils.computeVisionHeuristics();
  	optimizer.run();
}