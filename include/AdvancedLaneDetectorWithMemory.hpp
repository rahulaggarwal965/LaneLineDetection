//
//  AdvancedLaneDetectorWithMemory.hpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/1/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#ifndef AdvancedLaneDetectorWithMemory_hpp
#define AdvancedLaneDetectorWithMemory_hpp

#include "cpuUtils.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "LaneLine.hpp"
#include "LaneLineHistory.hpp"

class AdvancedLaneDetectorWithMemory {
private:
    //First Four need more work
    cv::Mat objpts;
    cv::Mat imgpts;
    cv::Mat perspectiveMatrix;
    cv::Mat inversePerspectiveMatrix;
    
    //Window Information
    int slidingWindowsPerLine;
    int slidingWindowsHalfWidth;
    int slidingWindowRecenterThresh;
    
    //Size
    
    //Img Real World Stuff
    cv::Size imgDimensions;
    int laneWidthPx;
    int laneCenterPxPSP;
    cv::Point realWorldLaneSizeMeters;
    
    //Maybe pre-compute some data
    
    
    //Memory Stuff
    bool previousLeft = false;
    bool previousRight = false;
    LaneLine previousLeftLaneLine;
    LaneLine previousRightLaneLine;
    
    LaneLineHistory previousLeftLaneLines;
    LaneLineHistory previousRightLaneLines;
    
public:
    //Testing Constructor
    AdvancedLaneDetectorWithMemory(std::vector<cv::Point2f> srcPSP, std::vector<cv::Point2f> dstPSP, int slidingWindowsPerLine, int slidingWindowsHalfWidth, int slidingWindowRecenterThresh, cv::Size imgDimensions = cv::Size(1280, 720), int laneWidthPx = 400, int laneCenterPxPSP = 300, cv::Point realWorldLaneSizeMeters = cv::Point(32, 3.7));

    //Constructor
    AdvancedLaneDetectorWithMemory(cv::Mat objpts, cv::Mat imgpts, std::vector<cv::Point2f> srcPSP, std::vector<cv::Point2f> dstPSP, int slidingWindowsPerLine, int slidingWindowsHalfWidth, int slidingWindowRecenterThresh, cv::Size imgDimensions = cv::Size(1280, 720), int laneWidthPx = 600, int laneCenterPxPSP = 800, cv::Point realWorldLaneSizeMeters = cv::Point(32, 3.7));
    
    
    //Processing Stuff
    std::array<LaneLine, 2> computeLaneLines(cv::Mat warped);
    double computeCenterOffset(LaneLine leftLine, LaneLine rightLine);
    void processImage(cv::Mat &image);
    
    //Drawing Stuff
    void drawLaneLines(cv::Mat warped, cv::Mat &dst, LaneLine leftLine, LaneLine rightLine);
    void drawLaneArea(cv::Mat warped, cv::Mat &dst, LaneLine leftLine, LaneLine rightLine);
    void equalizeHist(cv::Mat image, cv::Mat &dst);
};

#endif /* AdvancedLaneDetectorWithMemory_hpp */
