//
//  main.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 6/26/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include "cpuUtils.hpp"
#include <iostream>
#include <stdio.h>
#include "LaneLine.hpp"
#include "LaneLineHistory.hpp"
#include "AdvancedLaneDetectorWithMemory.hpp"

int main(int argc, char** argv)
{
    cv::String imageNames[8];
    for (int i = 1; i < 7; i++) {
        imageNames[i-1] = cv::String("/Users/Infinity/Desktop/test_images/test" + std::to_string(i) + ".jpg");
        
    }
    
    imageNames[6] = cv::String("/Users/Infinity/Desktop/test_images/straight_lines1.jpg");
    imageNames[7] = cv::String("/Users/Infinity/Desktop/test_images/straight_lines2.jpg");
    cv::Mat straight_lines1 = cv::imread(imageNames[7], cv::IMREAD_COLOR);
    cv:: Mat test6 = cv::imread(imageNames[5], cv::IMREAD_COLOR);
    cv::Mat test5 = cv::imread(imageNames[4], cv::IMREAD_COLOR);
    if(test6.empty() || straight_lines1.empty() || test5.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }
    
    //Video testing
    //cv::VideoCapture cap(cv::String("/Users/Infinity/Desktop/test_images/project_video.mp4"));
    cv::VideoCapture cap(0);
    cv::Mat frame;

    
    if (!cap.isOpened()) {
        throw "Error when reading the file";
    }
    //std::vector<cv::Point2f> src_pts{cv::Point2f(210, 720 - 1), cv::Point2f(595, 450), cv::Point2f(690, 450), cv::Point2f(1110, 720 - 1)};
    //std::vector<cv::Point2f> dst_pts{cv::Point2f(200, 720 - 1), cv::Point2f(200, 0), cv::Point2f(1000, 0), cv::Point2f(1000, 720 - 1)};
    std::vector<cv::Point2f> src_pts{cv::Point2f(105, 359), cv::Point2f(297, 225), cv::Point2f(345, 225), cv::Point2f(555, 359)};
    std::vector<cv::Point2f> dst_pts{cv::Point2f(100, 359), cv::Point2f(100, 0), cv::Point2f(500, 0), cv::Point2f(500, 359)};
    cv::Mat M_psp = cv::getPerspectiveTransform(src_pts, dst_pts);
    //AdvancedLaneDetectorWithMemory ld(src_pts, dst_pts, 20, 100, 50);
    AdvancedLaneDetectorWithMemory ld(src_pts, dst_pts, 20, 100, 50, cv::Size(640, 360));
    cv::Mat channels[3];
    
    while(true) {

        cap >> frame;
        //std::cout << frame.size() << std::endl;
        if(frame.empty()) {
            break;
        }
        /*cv::resize(frame, frame, cv::Size(640, 360));
         cv::Mat test = cpu::combinedSobelThresholdImage(frame);
         cv::warpPerspective(test, test, M_psp, cv::Size(test.cols, test.rows));
         
         std::array<LaneLine, 2> laneLines = ld.computeLaneLines(test);
         
         ld.drawLaneLines(test, frame, laneLines[0], laneLines[1]);*/
        ld.processImage(frame);

        cv::imshow("Display Window", frame);
        if(cv::waitKey(1) >= 0) {
            break;
        }
    }
    cv::waitKey(0);
    return 0;
}

