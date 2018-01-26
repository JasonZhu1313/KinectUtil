//
//  ProjectUtil.hpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/11/18.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#ifndef ProjectUtil_hpp
#define ProjectUtil_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

class ProjectUtil{
   
public:
    Mat depthIntrinsic;
    Mat rgbIntrinsic;
    Mat rotation;
    Mat transformation;
    Mat rgbImage;
    Mat translation;
    int projectToPointCloud(cv::Mat &depth,vector<Point3d> &pointCloud);
    int savePointCloud(std::string filePath,vector<Point3d> &pointCloud,vector<Point2d> &rgbImagePoint);
    int fromDepthSpaceToColorSpace(vector<Point3d> &depthSpace,vector<Point3d>&colorSpace);
    int correspondingColorCoordinates(vector<Point3d> &depthSpacePoint,vector<Point2d> &rgbImagePoint);
    void flipAndSaveAllTheImage();
    void init();
private:
    
};
#endif /* ProjectUtil_hpp */
