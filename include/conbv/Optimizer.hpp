#pragma once

#include "libcmaes/cmaes.h"
#include "conbv/visionUtils.hpp"
#include <iostream>

using namespace libcmaes;

class Optimizer {
public:
    Optimizer(int &nDrones_, std::vector<cv::Point3f>& mapPoints_) : nDim(nDrones_*4), visionUtils(nDrones_, mapPoints_)
    {
        map = &mapPoints_;
    }

    FitFunc fsphere = [](const double *x, const int N)
    {
        double val = 0.0;
        for (int i=0;i<N;i++)
            val += x[i]*x[i];
        return val; 
    };

    void setupProblem()
    {
        visionUtils.computeProjections(*map);
        visionUtils.computeVisionHeuristics();
    }

    int run()
    {
        int dim = 10; // problem dimensions.
  	    std::vector<double> x0(dim,10.0);
  	    double sigma = 0.1;
  	    //int lambda = 100; // offsprings at each generation.
  	    CMAParameters<> cmaparams(x0,sigma);
  	    //cmaparams.set_algo(BIPOP_CMAES);
  	    CMASolutions cmasols = cmaes<>(fsphere,cmaparams);
  	    std::cout << "best solution: " << cmasols << std::endl;
  	    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
  	    return cmasols.run_status();
    }

private:
    int nDim;
    std::vector <cv::Point3f> *map;
    VisionUtils visionUtils;
};