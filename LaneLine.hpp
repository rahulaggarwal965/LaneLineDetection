//
//  LaneLine.hpp
//  opencv4_test
//
//  Created by Rahul Aggarwal on 6/30/19.
//  Copyright © 2019 Rahul Aggarwal. All rights reserved.
//

#ifndef LaneLine_hpp
#define LaneLine_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

class LaneLine {
private:
    std::array<double, 3> polynomialCoeff;
    std::vector<double> lineFitX;
    std::vector<int> nonzeroX;
    std::vector<int> nonzeroY;
    std::vector<std::array<cv::Point, 2>> windows;
public:
    //Constructor
    //LaneLine(int vSize);
    LaneLine();
    
    //Getters and Setters
    std::array<double, 3> getPolynomialCoeffIndex() { return this->polynomialCoeff; }
    double getPolynomialCoeffIndex(int i) { return this->polynomialCoeff[i]; }
    void setPolynomialCoeff(std::array<double, 3> polynomialCoeff) {this->polynomialCoeff = polynomialCoeff; }
    
    
    std::vector<double> getLineFitX() { return this->lineFitX; }
    double getLineFitXIndex(int i) { return this->lineFitX[i]; }
    void setLineFitX(std::vector<double> lineFitX) { this->lineFitX = lineFitX; }
    
    
    std::vector<int> getNonzeroX() { return this->nonzeroX; }
    void setNonzeroX(std::vector<int> nonzeroX ) { this->nonzeroX = nonzeroX; }
    std::vector<int> getNonxeroY() { return this->nonzeroY; }
    void setNonzeroY(std::vector<int> nonzeroY) {this->nonzeroY = nonzeroY; }
    
    std::vector<std::array<cv::Point, 2>> getWindows() {return this->windows; }
    
    //Actual Functions
    void appendWindow(std::array<cv::Point, 2> window);
    
    
    
};

#endif /* LaneLine_hpp */
