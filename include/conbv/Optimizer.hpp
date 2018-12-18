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

    FitFunc fvision = [&](const double *x, const int N)
    {
        visionUtils.computeProjections(x, N, *map);
        double val = 1 - visionUtils.heuristics.visibility; // + 100*visionUtils.heuristics.span;

        std::cout << "VISIBILITY is " << val << std::endl;
        // std::cout << "Span factor is " << visionUtils.heuristics.span << std::endl;
        return val;
    };

    void setupProblem()
    {
        // visionUtils.computeProjections(*map);        
        // visionUtils.computeVisionHeuristics();
    }

    int run()
    {
        int dim = 8; // problem dimensions.
  	    std::vector<double> x0(dim, 0.0);
  	    double sigma = 5;
  	    int lambda = 100; // offsprings at each generation.
  	    CMAParameters<> cmaparams(x0, sigma, lambda);
  	    //cmaparams.set_algo(BIPOP_CMAES);
  	    CMASolutions cmasols = cmaes<>(fvision, cmaparams);
  	    std::cout << "best solution: " << cmasols << std::endl;
  	    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
  	    return cmasols.run_status();
    }

private:
    int nDim;
    std::vector <cv::Point3f> *map;
    VisionUtils visionUtils;
};