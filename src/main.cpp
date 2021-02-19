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
#include "opencv2/videoio.hpp"

int main(int argc, char** argv)
{

    cv::VideoCapture cap;
    if (argc > 1) {
        printf("Running test video: %s\n", argv[1]);
        cap.open(argv[1]);
    } else {
        printf("Running on webcam:\n");
        cap.open(0);
    }
    cv::Mat frame;
    int fps = cap.get(cv::CAP_PROP_FPS);

    if (!cap.isOpened()) {
        throw "Error when reading the file";
    }

    std::vector<cv::Point2f> src_pts{cv::Point2f(105, 359), cv::Point2f(297, 225), cv::Point2f(345, 225), cv::Point2f(555, 359)};
    std::vector<cv::Point2f> dst_pts{cv::Point2f(100, 359), cv::Point2f(100, 0), cv::Point2f(500, 0), cv::Point2f(500, 359)};
    cv::Mat M_psp = cv::getPerspectiveTransform(src_pts, dst_pts);
    AdvancedLaneDetectorWithMemory ld(src_pts, dst_pts, 20, 100, 50, cv::Size(640, 360));
    cv::Mat channels[3];

    cv::VideoWriter writer;
    if (argc > 2) {
        printf("Writing to output video: output_videos/%s\n", argv[2]);
        writer.open(argv[2], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, ld.imgDimensions);
    } else {
        printf("Writing to output video: output_videos/out.avi\n");
        writer.open("output_videos/out.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, ld.imgDimensions);
    }

    while(cap.isOpened()) {

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
        writer << frame;

        cv::imshow("Display Window", frame);
        if(cv::waitKey(15) == 113) {
            break;
            writer.release();
        }
    }
}

