//
//  AdvancedLaneDetectorWithMemory.cpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 7/1/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#include "AdvancedLaneDetectorWithMemory.hpp"

//Testing Constructor **DELETE LATER**
AdvancedLaneDetectorWithMemory::AdvancedLaneDetectorWithMemory(std::vector<cv::Point2f> srcPSP, std::vector<cv::Point2f> dstPSP, int slidingWindowsPerLine, int slidingWindowsHalfWidth, int slidingWindowRecenterThresh, cv::Size imgDimensions, int laneWidthPx, int laneCenterPxPSP, cv::Point realWorldLaneSizeMeters) {
    this->perspectiveMatrix = cv::getPerspectiveTransform(srcPSP, dstPSP);
    this->inversePerspectiveMatrix = cv::getPerspectiveTransform(dstPSP, srcPSP);
    this->slidingWindowsPerLine = slidingWindowsPerLine;
    this->slidingWindowsHalfWidth = slidingWindowsHalfWidth;
    this->slidingWindowRecenterThresh = slidingWindowRecenterThresh;
    this->imgDimensions = imgDimensions;
    this->laneWidthPx = laneWidthPx;
    this->laneCenterPxPSP = laneCenterPxPSP;
    this->realWorldLaneSizeMeters = realWorldLaneSizeMeters;
}

//Constructor
AdvancedLaneDetectorWithMemory::AdvancedLaneDetectorWithMemory(cv::Mat objpts, cv::Mat imgpts, std::vector<cv::Point2f> srcPSP, std::vector<cv::Point2f> dstPSP, int slidingWindowsPerLine, int slidingWindowsHalfWidth, int slidingWindowRecenterThresh, cv::Size imgDimensions, int laneWidthPx, int laneCenterPxPSP, cv::Point realWorldLaneSizeMeters) {
    this->objpts = objpts;
    this->imgpts = imgpts;
    this->perspectiveMatrix = cv::getPerspectiveTransform(srcPSP, dstPSP);
    this->inversePerspectiveMatrix = perspectiveMatrix.inv();
    this->slidingWindowsPerLine = slidingWindowsPerLine;
    this->slidingWindowsHalfWidth = slidingWindowsHalfWidth;
    this->slidingWindowRecenterThresh = slidingWindowRecenterThresh;
    this->imgDimensions = imgDimensions;
    this->laneWidthPx = laneWidthPx;
    this->laneCenterPxPSP = laneCenterPxPSP;
    this->realWorldLaneSizeMeters = realWorldLaneSizeMeters;
}

void AdvancedLaneDetectorWithMemory::processImage(cv::Mat &image) {
    cv::resize(image, image, this->imgDimensions);
    cv::Mat temp;
    //this->equalizeHist(image, temp);
    cv::warpPerspective(image, temp, this->perspectiveMatrix, cv::Size(image.cols, image.rows));
    cpu::combinedSobelThresholdImage(temp, temp);
    //cv::warpPerspective(temp, temp, this->perspectiveMatrix, cv::Size(temp.cols, temp.rows));
    
    
    std::array<LaneLine, 2> laneLines = this->computeLaneLines(temp);
    //std::cout << this->computeCenterOffset(laneLines[0], laneLines[1]) << std::endl;
    
    this->drawLaneArea(temp, image, laneLines[0], laneLines[1]);
    //this->drawLaneLines(temp, image, laneLines[0], laneLines[1]);
    
    this->previousLeftLaneLine = laneLines[0];
    this->previousLeft = true;
    this->previousLeftLaneLine = laneLines[1];
    this->previousRight = true;
    
}

//Compute the Lane Lines for a given warp image
std::array<LaneLine, 2> AdvancedLaneDetectorWithMemory::computeLaneLines(cv::Mat warped) {
    cv::Mat hist;
    cv::reduce(warped(cv::Range(warped.rows/2, warped.rows), cv::Range(0, warped.cols)), hist, 0, cv::REDUCE_SUM, CV_64FC1);
    cv::Point leftXBase, rightXBase;
    minMaxLoc(hist.colRange(0, hist.cols/2), NULL, NULL, NULL, &leftXBase);
    minMaxLoc(hist.colRange(hist.cols/2, hist.cols), NULL, NULL, NULL, &rightXBase);
    rightXBase.x += hist.cols/2;
    
    cv::Mat nonzeros;
    cv::findNonZero(warped, nonzeros);
    double nonZeroFoundPCT = 0.0;
    
    LaneLine leftLine;
    LaneLine rightLine;
    
    double nonZeroFound = 0;
    std::vector<int> leftX;
    std::vector<int> rightX;
    std::vector<int> leftY;
    std::vector<int> rightY;
    
    if (this->previousLeft == true && this->previousRight == true) {
        /*nonzeros.forEach<cv::Point_<int>>([this, &nonZeroFound, &leftX, &leftY, &rightX, &rightY](cv::Point_<int> &p, const int * position) -> void {
         if((p.x > (this->previousLeftLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousLeftLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousLeftLaneLine.getPolynomialCoeffIndex(2) - this->slidingWindowsHalfWidth)) && (p.x < (this->previousLeftLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousLeftLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousLeftLaneLine.getPolynomialCoeffIndex(2) + this->slidingWindowsHalfWidth))) {
         leftX.push_back(p.x);
         leftY.push_back(p.y);
         nonZeroFound++;
         }
         if((p.x > (this->previousRightLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousRightLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousRightLaneLine.getPolynomialCoeffIndex(2) - this->slidingWindowsHalfWidth)) && (p.x < (this->previousRightLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousRightLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousRightLaneLine.getPolynomialCoeffIndex(2) + this->slidingWindowsHalfWidth))) {
         rightX.push_back(p.x);
         rightY.push_back(p.y);
         nonZeroFound++;
         }
         });*/
        for (int i = 0; i < nonzeros.rows; i++ ) {
            cv::Point_<int> p = nonzeros.at<cv::Point_<int>>(i);
            if((p.x > (this->previousLeftLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousLeftLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousLeftLaneLine.getPolynomialCoeffIndex(2) - this->slidingWindowsHalfWidth)) && (p.x < (this->previousLeftLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousLeftLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousLeftLaneLine.getPolynomialCoeffIndex(2) + this->slidingWindowsHalfWidth))) {
                leftX.push_back(p.x);
                leftY.push_back(p.y);
                nonZeroFound += 1;
            }
            if((p.x > (this->previousRightLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousRightLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousRightLaneLine.getPolynomialCoeffIndex(2) - this->slidingWindowsHalfWidth)) && (p.x < (this->previousRightLaneLine.getPolynomialCoeffIndex(0) * std::pow(p.y, 2) + this->previousRightLaneLine.getPolynomialCoeffIndex(1) * p.y + this->previousRightLaneLine.getPolynomialCoeffIndex(2) + this->slidingWindowsHalfWidth))) {
                rightX.push_back(p.x);
                rightY.push_back(p.y);
                nonZeroFound += 1;
            }
        }
        nonZeroFoundPCT = (nonZeroFound/nonzeros.rows);
        //std::cout << nonZeroFoundPCT << std::endl;
    }
    
    if(nonZeroFoundPCT < 0.85) {
        leftX.clear();
        rightX.clear();
        leftY.clear();
        rightY.clear();
        int windowHeight = warped.rows/this->slidingWindowsPerLine;
        for (int i = 0; i < this->slidingWindowsPerLine; i++) {
            int winYLow = warped.rows - (i + 1) * windowHeight;
            int winYHigh = warped.rows - i * windowHeight;
            
            int winXLeftLow = leftXBase.x - this->slidingWindowsHalfWidth;
            int winXLeftHigh = leftXBase.x + this->slidingWindowsHalfWidth;
            int winXRightLow = rightXBase.x - this->slidingWindowsHalfWidth;
            int winXRightHigh = rightXBase.x + this->slidingWindowsHalfWidth;
            
            leftLine.appendWindow(std::array<cv::Point, 2>{cv::Point(winXLeftLow, winYLow), cv::Point(winXLeftHigh, winYHigh)});
            rightLine.appendWindow(std::array<cv::Point, 2>{cv::Point(winXRightLow, winYLow), cv::Point(winXRightHigh, winYHigh)});
            
            //PARALLEL ACCESS TO MATRIX CAUSES VECTOR TO BE WRITTEN TO AT THE SAME TIME
            //TODO: WRITE FUNCTION ***IN PLACE*** FOR MATRIX SO AS TO SPEED UP ACCESS
            
            /*            nonzeros.forEach<cv::Point_<int>>([winYLow, winYHigh, winXLeftLow, winXRightLow, winXLeftHigh, winXRightHigh, &nonZeroFound, &leftX, &leftY, &rightX, &rightY](cv::Point_<int> &p, const int * position) -> void {
             if ((p.y >= winYLow) && (p.y <= winYHigh) && (p.x >= winXLeftLow) && p.x <= winXLeftHigh) {
             leftX.push_back(p.x);
             leftY.push_back(p.y);
             nonZeroFound++;
             }
             if ((p.y >= winYLow) && (p.y <= winYHigh) && (p.x >= winXRightLow) && p.x <= winXRightHigh) {
             rightX.push_back(p.x);
             rightY.push_back(p.y);
             nonZeroFound++;
             }
             });*/
            
            for (int i = 0; i < nonzeros.rows; i++) {
                cv::Point_<int> p = nonzeros.at<cv::Point_<int>>(i);
                if ((p.y >= winYLow) && (p.y <= winYHigh) && (p.x >= winXLeftLow) && p.x <= winXLeftHigh) {
                    leftX.push_back(p.x);
                    leftY.push_back(p.y);
                    nonZeroFound += i;
                }
                if ((p.y >= winYLow) && (p.y <= winYHigh) && (p.x >= winXRightLow) && p.x <= winXRightHigh) {
                    rightX.push_back(p.x);
                    rightY.push_back(p.y);
                    nonZeroFound += i;
                }
            }
            
            if (leftX.size() > this->slidingWindowRecenterThresh) {
                leftXBase.x = cv::mean(leftX)[0];
            }
            if (rightX.size() > this->slidingWindowRecenterThresh) {
                rightXBase.x = cv::mean(rightX)[0];
            }
        }
        nonZeroFoundPCT = (nonZeroFound/nonzeros.rows);
    }
    
    std::array<double, 3> leftFit, rightFit;
    leftFit = cpu::polyFitDegree2(leftY, leftX);
    rightFit = cpu::polyFitDegree2(rightY, rightX);
    leftLine.setPolynomialCoeff(leftFit);
    rightLine.setPolynomialCoeff(rightFit);
    
    if(!this->previousLeftLaneLines.append(leftLine)) {
        leftFit = this->previousLeftLaneLines.smoothedPolynomial();
        leftLine.setPolynomialCoeff(leftFit);
        this->previousLeftLaneLines.append(leftLine, true);
    }
    
    if(!this->previousRightLaneLines.append(rightLine)) {
        rightFit = this->previousRightLaneLines.smoothedPolynomial();
        rightLine.setPolynomialCoeff(rightFit);
        this->previousRightLaneLines.append(rightLine, true);
    }
    
    std::vector<double> leftFitX(this->imgDimensions.width), rightFitX(this->imgDimensions.width);
    for (int i = 0; i < imgDimensions.width; i++) {
        leftFitX[i] = leftFit[0] * (double) std::pow(i, 2) + leftFit[1] * (double) i + leftFit[2];
        rightFitX[i] = rightFit[0] * (double) std::pow(i, 2) + rightFit[1] * (double) i + rightFit[2];
    }
    
    leftLine.setPolynomialCoeff(leftFit);
    leftLine.setLineFitX(leftFitX);
    leftLine.setNonzeroX(leftX);
    leftLine.setNonzeroY(leftY);
    
    rightLine.setPolynomialCoeff(rightFit);
    rightLine.setLineFitX(rightFitX);
    rightLine.setNonzeroX(rightX);
    rightLine.setNonzeroY(rightY);
    
    return std::array<LaneLine, 2>{leftLine, rightLine};
}

void AdvancedLaneDetectorWithMemory::drawLaneLines(cv::Mat warped, cv::Mat &dst, LaneLine leftLine, LaneLine rightLine) {
    cv::Mat out;
    cv::Mat channels[3] = {warped, warped, warped};
    cv::merge(channels, 3, out);
    std::vector<cv::Point> ptsLeft, ptsRight;
    for (int i = 0; i < warped.rows; i++) {
        ptsLeft.push_back(cv::Point((int) leftLine.getLineFitXIndex(i), i));
        ptsRight.push_back(cv::Point((int) rightLine.getLineFitXIndex(i), i));
    }
    
    //Drawing the Lines
    cv::polylines(out, std::vector<std::vector<cv::Point>> {ptsLeft}, false, cv::Scalar(0, 140, 255), 5);
    cv::polylines(out, std::vector<std::vector<cv::Point>> {ptsRight}, false, cv::Scalar(0, 140, 255), 5);
    
    for (auto const& window: leftLine.getWindows()) {
        cv::rectangle(out, window[0], window[1], cv::Scalar(0, 255, 0), 3);
    }
    for (auto const& window: rightLine.getWindows()) {
        cv::rectangle(out, window[0], window[1], cv::Scalar(0, 255, 0), 3);
    }
    dst = out;
}

void AdvancedLaneDetectorWithMemory::drawLaneArea(cv::Mat warped, cv::Mat &dst, LaneLine leftLine, LaneLine rightLine) {
    cv::Mat warpedZero = cv::Mat::zeros(warped.rows, warped.cols, CV_8UC3);
    
    std::vector<cv::Point> ptsLeft, ptsRight;
    for (int i = 0; i < warped.rows; i++) {
        ptsLeft.push_back(cv::Point((int) leftLine.getLineFitXIndex(i), i));
        ptsRight.push_back(cv::Point((int) rightLine.getLineFitXIndex(i), i));
    }
    ptsLeft.insert(ptsLeft.end(), ptsRight.rbegin(), ptsRight.rend());
    cv::fillPoly(warpedZero, std::vector<std::vector<cv::Point>> {ptsLeft}, cv::Scalar(0, 255, 0));
    cv::warpPerspective(warpedZero, warpedZero, this->inversePerspectiveMatrix, cv::Size(dst.cols, dst.rows));
    cv::addWeighted(dst, 1, warpedZero, 0.3, 0, dst);
}

void AdvancedLaneDetectorWithMemory::equalizeHist(cv::Mat image, cv::Mat &dst) {
    cv::Mat channels[3];
    cv::cvtColor(image, image, cv::COLOR_BGR2YCrCb);
    cv::split(image, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, 3, image);
    cv::cvtColor(image, dst, cv::COLOR_YCrCb2BGR);
}

double AdvancedLaneDetectorWithMemory::computeCenterOffset(LaneLine leftLine, LaneLine rightLine) {
    return (((leftLine.getPolynomialCoeffIndex(0) * std::pow(this->imgDimensions.width - 1, 2) + leftLine.getPolynomialCoeffIndex(1) * (this->imgDimensions.width - 1) + leftLine.getPolynomialCoeffIndex(2)) + ( rightLine.getPolynomialCoeffIndex(0) * std::pow(this->imgDimensions.width - 1, 2) + rightLine.getPolynomialCoeffIndex(1) * (this->imgDimensions.width - 1) + rightLine.getPolynomialCoeffIndex(2)) / 2) - this->laneCenterPxPSP) * 3.7/400;
}
