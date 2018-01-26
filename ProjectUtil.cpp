//
//  ProjectUtil.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/11/18.
//  Copyright © 2017年 思宇朱. All rights reserved.
//

#include "ProjectUtil.hpp"

void ProjectUtil::flipAndSaveAllTheImage(){
    for(int i =1;i<=20;i++){
        std::string imagepath = "/Users/siyuzhu/Public/hku_lab/mismatch/Calibrari/irimage"+to_string(i)+".bmp";
        std::string outputPath ="/Users/siyuzhu/Public/hku_lab/mismatch/CalibFlipir/irimage"+to_string(i)+".bmp";
        Mat image = imread(imagepath);
        Mat resultimage;
        cv::flip(image, resultimage, 0);
        imwrite(outputPath, resultimage);
    }
    for(int i =1;i<=20;i++){
        std::string imagepath = "/Users/siyuzhu/Public/hku_lab/mismatch/Calibrargb/rgbimage"+to_string(i)+".bmp";
        std::string outputPath ="/Users/siyuzhu/Public/hku_lab/mismatch/CalibFliprgb/rgbimage"+to_string(i)+".bmp";
        Mat image = imread(imagepath);
        Mat resultimage;
        cv::flip(image, resultimage, 0);
        imwrite(outputPath, resultimage);
    }
    cout<<"finishing flip"<<endl;
}




void ProjectUtil::init(){
    depthIntrinsic =(Mat_ <float>(3,3) << 591.81690,0,316.77227, 0,591.23953,249.56119, 0,0,1);
    rgbIntrinsic=(Mat_<float>(3,3) <<   516.27251,0,319.21872, 0,516.52227, 225.29767, 0,0,1   );
    rotation =(Mat_ <float>(3,3) <<   1.0000,   -0.0009,    0.0006,
               0.0009,    0.9999,    0.0134
               -0.0006,   -0.0134,    0.9999 );
//     rotation =(Mat_ <float>(3,3) <<     1.0000,   -0.0015,   0.0053,
//                          0.0015,    0.9999,   -0.0136,
//                          -0.0053,    0.0136,  0.9999 );
    transformation=(Mat_<float>(4,4)<< 1.0000,   -0.0009,   0.0006, -25.36961,
                                       0.0009,    0.9999,    0.0134, -1.15194,
                                       -0.0006,   -0.0134,    0.9999,  -7.98245,
                                             0,         0,         0,    1);
    
   // 25.61034   -0.21963  3.28608
    //-25.36961   -1.15194  -7.98245  fix
    cout<<transformation<<endl;
    Mat rgbImageraw=imread("/Users/siyuzhu/Public/hku_lab/mismatch/Calibrari/mismatch/Data/RGB/mycolor/color2.bmp");
    cv::flip(rgbImageraw, this->rgbImage, 1);
}

int ProjectUtil:: correspondingColorCoordinates(vector<Point3d> &depthSpacePoint,vector<Point2d> &rgbImagePoint){
    bool flag=false;
    vector<Point3d> colorSpace;
    if(depthSpacePoint.size()>0){
        fromDepthSpaceToColorSpace(depthSpacePoint,colorSpace);
    }
    for(int i =0;i<colorSpace.size();i++){
        Mat colorSpaceMat = (Mat_<float> (3,1)<< colorSpace[i].x,colorSpace[i].y,colorSpace[i].z);
        Mat result = rgbIntrinsic * colorSpaceMat;
        float x =result.at<float>(0,0)/result.at<float>(2,0);
        float y =result.at<float>(1,0)/result.at<float>(2,0);
        rgbImagePoint.push_back(Point2d(x,y));
    }
    return 1;
}

int ProjectUtil::fromDepthSpaceToColorSpace(vector<Point3d> &depthSpace,vector<Point3d> &colorSpace){
    Point3d colorPoint;
    if(rotation.rows!=3||rotation.cols!=3){
        cout<<"rotation matrix error"<<endl;
        return -1;
    }
    if(transformation.rows!=4||transformation.cols!=4){
        cout<<"transformation matrix error"<<endl;
        return -1;
    }
    
   
    cout<<"tranlation"<<transformation<<endl;
    cout<<"rotation"<<rotation<<endl;
    for(int i=0;i<depthSpace.size();i++){
        Point3d depthPoint=depthSpace[i];
        float px = depthPoint.x;
        float py = depthPoint.y;
        float pz = depthPoint.z;
        Mat temp = (Mat_ <float>(4,1) <<px,py,pz,1);
        Mat result = transformation * temp;
        float colorx = result.at<float>(0,0);
        float colory = result.at<float>(0,1);
        float colorz = result.at<float>(0,2);
        colorPoint.x=colorx;
        colorPoint.y=colory;
        colorPoint.z=colorz;
        colorSpace.push_back(colorPoint);
    }
    return 1;
}


int ProjectUtil:: projectToPointCloud( cv::Mat &depth,vector<Point3d> &pointCloud){
    float fx=depthIntrinsic.at<float>(0,0);
    float fy=depthIntrinsic.at<float>(1,1);
    float cx=depthIntrinsic.at<float>(0,2);
    float cy=depthIntrinsic.at<float>(1,2);
    if(depthIntrinsic.cols!=3||depthIntrinsic.rows!=3){
        cout<<"intrinsic size error"<<endl;
        return -1;
    }
   
    for(int i = 0;i< depth.rows;i++){
        for(int j =0;j< depth.cols;j++){
            unsigned short z = depth.at<unsigned short>(i,j);
            //float zfinal = z*1e-3;
            float zfinal = z;
            if(z>0){
                //x,y in the depth camera coordinate
                float x = (j*zfinal - cx*zfinal)/fx;
                float y = (i*zfinal - cy*zfinal)/fy;
                Point3d p;
                p.x=x;
                p.y=y;
                p.z=zfinal;
                pointCloud.push_back(p);
            }
        }
    }
    return 1;
}

int ProjectUtil::savePointCloud(std::string filePath, vector<Point3d> &pointCloud,vector<Point2d> &rgbImagePoint){
    std::ofstream fileout(filePath);
    fileout << "COFF" << std::endl;
    fileout << pointCloud.size() << " 0 0" << std::endl;
    for (int i = 0; i < pointCloud.size(); i++)
    {
        Point2d colorPoint=rgbImagePoint[i];
        if(colorPoint.x>=0 && colorPoint.x<640 && colorPoint.y>=0 && colorPoint.y<480){
            Vec3b color;
            color = rgbImage.at<Vec3b>(colorPoint);
            fileout << pointCloud[i].x << " " << pointCloud[i].y << " " << pointCloud[i].z << " " <<(int)color[2] << " " <<(int)color[1]<< " " <<(int)color[0] << " 255" << std::endl;
        }else{
            fileout << pointCloud[i].x << " " << pointCloud[i].y << " " << pointCloud[i].z << " " <<0 << " " <<0<< " " <<0 << " 255" << std::endl;
           
        }
    }
    
    fileout.close();
    return 1;
}






