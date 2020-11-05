//
//  LaneLineHistory.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/1/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include "LaneLineHistory.hpp"


//Constructor
LaneLineHistory::LaneLineHistory(std::array<int, 4> testPoints, double polyMaxDeviationDistance) {
    this->testPoints = testPoints;
    this->polyMaxDeviationDistance = polyMaxDeviationDistance;
}

//Append a LaneLine to the LaneLineHistory Deque that returns true if it is added
bool LaneLineHistory::append(LaneLine laneLine, bool force) {
    if (this->laneLines.size() == 0 || force == true) {
        this->laneLines.push_front(laneLine);
        if(this->laneLines.size() > 10) {
            this->laneLines.pop_back();
        }
        this->smoothedPolynomial();
        return true;
    }
    cv::Mat testYSmooth(1, 4, CV_64FC1);
    cv::Mat testYNew(1, 4, CV_64FC1);
    cv::Mat dist(1, 4, CV_64FC1);
    
    for (int i = 0; i < 4; i++) {
        testYSmooth.at<double>(i) = this->smoothedPoly[0] * (double) std::pow(this->testPoints[i], 2) + this->smoothedPoly[1] * (double) this->testPoints[i] + this->smoothedPoly[2];
        testYNew.at<double>(i) = laneLine.getPolynomialCoeffIndex(0) * (double) std::pow(this->testPoints[i], 2) + laneLine.getPolynomialCoeffIndex(1) * (double) this->testPoints[i] + laneLine.getPolynomialCoeffIndex(2);
        dist.at<double>(i) = abs(testYSmooth.at<double>(i) - testYNew.at<double>(i));
    }
    double max_dist;
    cv::minMaxLoc(dist, NULL, &max_dist);
    
    if (max_dist > this->polyMaxDeviationDistance) {
        //std::cerr << "MAX DISTANCE BREACHED" << std::endl;
        return false;
    }
    
    this->laneLines.push_front(laneLine);
    if(this->laneLines.size() > 10) {
        this->laneLines.pop_back();
    }
    this->smoothedPolynomial();
    
    
    return true;
}

//Return a smoothed polynomial
std::array<double, 3> LaneLineHistory::smoothedPolynomial() {
    cv::Mat smoothedPolyMat((int) this->laneLines.size(), 3, CV_64F);
    smoothedPolyMat.forEach<double>([this](double &p, const int * position) -> void {
        p = this->laneLines[position[0]].getPolynomialCoeffIndex(position[1]);
    });
    cv::reduce(smoothedPolyMat, smoothedPolyMat, 0, cv::REDUCE_AVG, CV_64F);
    std::array<double, 3> smoothedPolyArray;
    for (int i = 0; i < 3; i++) {
        smoothedPolyArray[i] = smoothedPolyMat.at<double>(i);
    }
    this->smoothedPoly = smoothedPolyArray;
    return smoothedPolyArray;
}
