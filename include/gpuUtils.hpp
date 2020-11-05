//
//  gpuUtils.hpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/7/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#ifndef gpuUtils_hpp
#define gpuUtils_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace gpu {
    cv::Mat rgb_to_hls (cv::Mat image);
    
    cv::Mat rgb_to_lab (cv::Mat image);
    
    std::array<double, 3> polyFitDegree2(std::vector<int> left, std::vector<int> right);
    
    void colorThreshold(cv::Mat image, cv::Mat &dst);
    
    void convertAbsAxisSobel(cv::Mat sobel, cv::Mat &dst, cv::Scalar lowerBound=cv::Scalar(20), cv::Scalar upperBound = cv::Scalar(120));
    
    void magSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, int kernelSize, cv::Scalar lowerBound, cv::Scalar upperBound);
    
    void dirSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, cv::Scalar lowerBound, cv::Scalar upperBound);
    
    
    void combinedSobels(cv::Mat gray, cv::Mat &dst, int kernelSize);
    
    void get_combined_binary_thresholded_img(cv::Mat image, cv::Mat &dst);
    
    void combinedSobelThresholdImage(cv::Mat image, cv::Mat &dst);
}

#endif /* gpuUtils_hpp */
