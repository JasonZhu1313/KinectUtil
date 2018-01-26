//
//  directlineartransform.hpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/29.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#ifndef directlineartransform_hpp
#define directlineartransform_hpp

#include <stdio.h>
#include "directlineartransform.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>


using namespace std;
using namespace cv;
using namespace Eigen;



class DLT{
public:
    int init();
    //read the correct project points from file
    int readDataFromFile(std::string imagePointsPath,std::string objectPointsPath);
    int getPointCloudAndSave();
    VectorXf initEigen();
    int generatePointCloud(Mat colorMat,vector<Point3d> cameraSpacePoints,vector<Point2d> refinedPoint,std::string outputPath);
    //use the calculated para to project from 3d camera space point to 2d color space point
    vector<Point2d> reprojectToColorSpace(vector<Point3d> cameraSpacePoints);
    int save(std::string targetOffPath);
    int calculateParam();
    Mat getp();
    float calculateError(vector<Point2d> projectedPoint);
    

private:
    //A paramMat = B
    //3*4
    Mat p;
    Mat pline1;
    Mat pline2;
    Mat pline3;
    Mat paramMat;
    //11*1
    Mat A;
    //11*1
    Mat B;
    int imageCount=0;
    vector<Point2d> imagePoints;
    vector<Point3d> objectPoints;
   
    
};
#endif /* directlineartransform_hpp */
