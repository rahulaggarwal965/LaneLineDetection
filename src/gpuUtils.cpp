//
//  gpuUtils.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/7/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include "gpuUtils.hpp"

namespace gpu {
    /*cv::cuda::GpuMat rgb_to_hls (cv::cuda::GpuMat image) {
     cv::cuda::GpuMat hls;
     cv::cvtColor(image, hls, cv::COLOR_BGR2HLS);
     }*/
    
    cv::Mat rgb_to_lab (cv::Mat image);
    
    std::array<double, 3> polyFitDegree2(std::vector<int> left, std::vector<int> right);
    
    void colorThreshold(cv::Mat image, cv::Mat &dst);
    
    void convertAbsAxisSobel(cv::Mat sobel, cv::Mat &dst, cv::Scalar lowerBound, cv::Scalar upperBound);
    
    void magSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, int kernelSize, cv::Scalar lowerBound, cv::Scalar upperBound);
    
    void dirSobel(cv::Mat sx, cv::Mat sy, cv::Mat &dst, cv::Scalar lowerBound, cv::Scalar upperBound);
    
    
    void combinedSobels(cv::Mat gray, cv::Mat &dst, int kernelSize);
    
    void get_combined_binary_thresholded_img(cv::Mat image, cv::Mat &dst);
    
    void combinedSobelThresholdImage(cv::Mat image, cv::Mat &dst);
}
