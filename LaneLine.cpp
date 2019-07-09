//
//  LaneLine.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 6/30/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include "LaneLine.hpp"

//Constructor
LaneLine::LaneLine() {}

//Append a window (the bounds) for a sliding window to the lane line7
void LaneLine::appendWindow(std::array<cv::Point, 2> window) {
    this->windows.push_back(window);
}
