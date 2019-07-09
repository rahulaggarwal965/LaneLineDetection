//
//  LaneLineHistory.hpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/1/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#ifndef LaneLineHistory_hpp
#define LaneLineHistory_hpp

#include <stdio.h>
#include "LaneLine.hpp"
#include <opencv2/opencv.hpp>

class LaneLineHistory {
private:
    std::deque<LaneLine> laneLines;
    std::array<double, 3> smoothedPoly;
    std::array<int, 4>testPoints;
    double polyMaxDeviationDistance;
public:
    //Constructor
    LaneLineHistory(std::array<int, 4> testPoints = {50, 300, 500, 700}, double polyMaxDeviationDistance = 250);
    
    //Getters and Setters
    double getPolyMaxDeviationDistance() {return this->polyMaxDeviationDistance; }
    
    //Actual Functions
    bool append(LaneLine laneLine, bool force = false);
    std::array<double, 3> smoothedPolynomial();
    
};

#endif /* LaneLineHistory_hpp */
