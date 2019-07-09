//
//  cpuUtils.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/7/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include "cpuUtils.hpp"

namespace cpu {

    cv::Mat rgb_to_hls (cv::Mat image) {
        cv::Mat hls;
        cv::cvtColor(image, hls, cv::COLOR_BGR2HLS);
        return hls;
    }

    cv::Mat rgb_to_lab (cv::Mat image) {
        cv::Mat lab;
        cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);
        //std::cout << lab.type() << std::endl;
        return lab;
    }

    std::array<double, 3> polyFitDegree2(std::vector<int> left, std::vector<int> right) {
        std::array<double, 3> ret;
        cv::Mat xMat = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat yMat = cv::Mat::zeros(3, 1, CV_64FC1);
        for (std::vector<int>::size_type k = 0; k != left.size(); k++) {
            for (int i = 0; i < 3; i++) {
                yMat.at<double>(i) += (double) right[k] * std::pow(left[k], i);
                for (int j = 0; j < 3; j++) {
                    xMat.at<double>(i, j) += (double) std::pow(left[k], i + j);
                }
            }
        }
        cv::solve(xMat, yMat, ret);
        std::reverse(ret.begin(), ret.end());
        return ret;
    }

    void colorThreshold(cv::Mat image, cv::Mat &dst) {
        cv::Mat hls = rgb_to_hls(image);
        cv::Mat whiteThresh, yellowThresh;
        cv::inRange(hls, cv::Scalar(15, 30, 115), cv::Scalar(35, 204, 255), yellowThresh);
        cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(255, 255, 255), whiteThresh);
        dst = yellowThresh | whiteThresh;
    }

    void convertAbsAxisSobel(cv::Mat sobel, cv::Mat &dst, cv::Scalar lowerBound, cv::Scalar upperBound) {
        double minSobel, maxSobel;
        cv::minMaxLoc(sobel, &minSobel, &maxSobel);
        sobel.convertTo(sobel, CV_8UC1, 255.0/((std::abs(maxSobel) > std::abs(minSobel)) ? std::abs(maxSobel) : std::abs(minSobel)));
        cv::inRange(sobel, lowerBound, upperBound, dst);
    }

    void magSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, int kernelSize, cv::Scalar lowerBound, cv::Scalar upperBound) {
        cv::Mat sxy;
        cv::magnitude(sx, sy, sxy);
        double maxSXY;
        cv::minMaxLoc(sxy, NULL, &maxSXY);
        sxy.convertTo(sxy, CV_8UC1, 255.0/maxSXY);
        cv::inRange(sxy, lowerBound, upperBound, dst);
    }

    void dirSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, cv::Scalar lowerBound, cv::Scalar upperBound) {
        cv::Mat dirSXY(sx.rows, sx.cols, CV_64FC1);
        dirSXY.forEach<double>([sx, sy](double &p, const int * position) -> void {
            p = cv::fastAtan2(cv::abs(sx.at<double>(position[0], position[1])), cv::abs(sy.at<double>(position[0], position[1])));
        });
        cv::inRange(dirSXY, lowerBound, upperBound, dst);
    }

    void combinedSobels(cv::Mat gray, cv::Mat &dst, int kernelSize) {
        cv::Mat sx, sy, sxy, dirSXY;
        cv::Sobel(gray, sx, CV_64FC1, 1, 0, kernelSize);
        cv::Sobel(gray, sy, CV_64FC1, 0, 1, kernelSize);
        
        magSobel(sx, sy, sxy, kernelSize, cv::Scalar(80), cv::Scalar(200));
        dirSobel(sx, sy, dirSXY, cv::Scalar(M_PI_4), cv::Scalar(M_PI_2));
        convertAbsAxisSobel(sx, sx, cv::Scalar(20), cv::Scalar(120));
        convertAbsAxisSobel(sy, sy, cv::Scalar(20), cv::Scalar(120));
        dst = sx | (sy & sxy & dirSXY);
    }

    void combinedSobelThresholdImage(cv::Mat image, cv::Mat &dst) {
        cv::Mat channels[3];
        cv::split(rgb_to_lab(image), channels);
        combinedSobels(channels[0], channels[0], 15);
        cv::Mat colorThresh;
        colorThreshold(image, colorThresh);
        dst = channels[0] | colorThresh;
    }
}
