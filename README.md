# KinectUtil
This project is used to solve the mismatch problem of rgb and depth camera of Kinect camera in order to get high quality point cloud. For calibration stage, we use 2 method to calibrate the camera, using DLT and Zhangzhengyou‘s checkerboard to calculate the camera parameters, and using these parameters to project and retroject from the image captured by RGB and depth camera. 
# Some Basic Terminology 
## Point cloud:
   Point cloud is a kind of 3D representation, and It is composed by a series of orderless points with 3D coordinate values, we can use RGB and Depth camera to produce the point cloud, the image captured by RGB camera is used to provide color information, and The depth camera is used to provide depth information. 

![image.png](http://upload-images.jianshu.io/upload_images/7058214-9a253484b3868e87.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

## Parameters of camera model
   Camera model can be shown as follow, the process of projecting the 3D environment to the 2D image have 2 steps:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-791b372c06b013ad.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

* 1. Translate and Rotate the 3D points from the world space to camera space by multiplying extrinsic parameters [R T].

* 2. Project the 3D points in camera space to the 2D by multiplying the intrinsic parameter K.

![Intrinsic parameter.png](http://upload-images.jianshu.io/upload_images/7058214-0c0f2fc8568ac47a.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

If we have the intrinsic and extrinsic parameters of the camera we can also back project the 2D coordinate to 3D coordinate.

## Description of Pinhole model & Intrinsic
The above camera model we talk about is Pinhole model, the image can be projected to a 2D plane as blow.
![image.png](http://upload-images.jianshu.io/upload_images/7058214-3c11f0a60c784f6d.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

According to the triangle similarity relation, there is

![image.png](http://upload-images.jianshu.io/upload_images/7058214-5c43541c63825d91.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
The difference between the pixel coordinate system and the imaging plane is a scale and shift：

![image.png](http://upload-images.jianshu.io/upload_images/7058214-51ebca92b6928ed8.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
![image.png](http://upload-images.jianshu.io/upload_images/7058214-5d375ebdacfe4c5c.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
Among the equation:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-438d85666f64d4d0.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
So, finally we get:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-12438b3a72128d5b.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
It can be wrote to the matrix format:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-04a472acb2d4e356.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

* f is the focal length of the lens

* The units of alpha and beta are pixels

* The focal length of fx and fy is the focal length in the direction of x and y, and the unit is pixel.

* (CX, CY) main point, the center of the image, unit as pixel

* fnormal_x,fnormal_y are Normalized focal length of X and Y in the direction of X and Y


# Why we need to calibrate the RGB and Depth Camera
## Circumstances 1:
Because the the rotation and translation of two cameras relative to each other are not perfect, so when generating the point cloud it will cause the color mismatch, which means the color image can be project to the wrong place and it will influence the quality of the point cloud. The mismatch of color is shown as below:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-0deb46f5a8f853d8.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

**The black color are not suppose to be the part of background**

![image.png](http://upload-images.jianshu.io/upload_images/7058214-83554d70d12f5ee1.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

==Kinect API encapsulates these parameters and the generating process into interface, and you can only invoke it as a black box, you can't see the implementation of the Kinect API which is not open source, so you can't know what happens inside the function, and it will generate the point cloud after you calling that interface. So we can't get access to the camera parameters provided by the manufacturer. If you are not satisfied with the quality of the point cloud, you need to calibrate by yourself.==

## Circumstances 2:
If the resolution and quality of RGB image captured by Kinect RGB camera can not reach you expectation, you can block the original RGB camera and attach a high resolution RGB camera of yours to the Kinect, under this circumstances , you much  calibrate the parameters and generate point cloud by yourself.

# How Can We Do That:
The process of calibration and generating point cloud can be simplified as:

* 1. Using calibration tools to calibrate the RGB and Depth cameras, get the intrinsic parameters of RGB camera, intrinsic parameters of Depth camera, and the relative translation and rotation of RGB camera to the depth camera.

* 2. Back project the RGB image to RGB camera space by using the intrinsic parameters of RGB camera.

* 3. Back project the Depth image to Depth camera space by using the intrinsic parameters of Depth camera.

* 4. Transform the RGB camera space to Depth camera space by using the rotation and translation of the RGB relative to the Depth camera.


### Detailed Calibration Step:
1) Get this matlab tool here:  http://www.vision.caltech.edu/bouguetj/calib_doc/
We would need to download TOOLBOX_calib zip folder and store it somewhere. Then just update our path in matlab that it will see it. 

2) Then we would need to get intrinsics following this example. 
http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example.html. We would need checkerboard detections (their are done in a semi-manual fashion) that have to be saved in the file as "Calib_Results.mat" (which store the detections and other information needed for extrinsics calibration.). Do this both for color and depth (ir) images separately. Store the images separately too.

**NOTE: don't forget to change default dX=dY=30mm=default values, to your size of the checkerboard.**

3) The infrared emitted by v1 will have lot of noise which will affect corner detection.

![image.png](http://upload-images.jianshu.io/upload_images/7058214-b2871e68de61f8a4.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

So we used the infrared emitted by v2 and I blocked the IR launcher of v1 which will cause Infrared speckle point that can interfere with corner detection. Or you can use the infrared emitted by some other light source. The experiment setting up  is shown as follow:

![image.png](http://upload-images.jianshu.io/upload_images/7058214-39642fe8ff915a5e.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


4) Then follow this example to get extrinsics: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example5.html. Then, set recompute_intrinsic_left = 0, recompute_intrinsic_right = 0. It makes sure intrinsic will not be changed when computing the extrinsics. Because the process of calculating the extrinsics can adjust the intrinsic parameters to lower the error, but we don't want the intrinsic to be changed.
The toolbox can restore the camera model, this is the two camera of my case.
![image.png](http://upload-images.jianshu.io/upload_images/7058214-8058ef2fa4de5755.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Calibration Result:

**NOTE: The intrinsic parameters and extrinsic parameters may vary from different Kinect Camera, the data calibrate by me just can serve as a reference.**

### Intrinsic parameters of RGB camera
%-- Focal length:
fc = [ 589.322232303107740 ; 589.849429472609130 ];

%-- Principal point:
cc = [ 321.140896612950880 ; 235.563195335248370 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.108983708314500 ; -0.239830795000911 ; -0.001984065259398 ; -0.002884597433015 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 9.099860081548242 ; 8.823165349428608 ];

%-- Principal point uncertainty:
cc_error = [ 5.021414252987091 ; 4.452250648666922 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.016941790847544 ; 0.054649912671684 ; 0.002535217040902 ; 0.003239597678439 ; 0.000000000000000 ];

### Intrinsic parameters of Depth camera
%-- Focal length:
fc = [ 458.455478616934780 ; 458.199272745572390 ];

%-- Principal point:
cc = [ 343.645038678435410 ; 229.805975111304460 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.127613248346941 ; 0.470606007302241 ; 0.000048478690145 ; 0.017448057052172 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 27.559874091123589 ; 26.899212523852725 ];

%-- Principal point uncertainty:
cc_error = [ 12.766795489113061 ; 9.541088535770328 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.065356450579064 ; 0.291607962509204 ; 0.006925403021099 ; 0.008487031884390 ; 0.000000000000000 ];

### Extrinsic parameters of Depth camera
#### Rotation vector:           
om = [ 0.00321   -0.05390  0.00084 ] +- [ 0.00262   0.00276  0.00238 ]
#### Translation vector:          
T = [ -19.94800   -0.74458  -10.84871 ]  +- [ 1.79424   1.68961  1.38079 ]



### Generate Point Cloud: 
After getting the camera parameters, I have written the interface of generating the point cloud, the ProjectUtil.cpp and ProjectUtil.hpp contains the point cloud generation process.

#### ProjectUtil.hpp

```

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
```

#### ProjectUtil.cpp

```
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

```

#### The Github of this project
https://github.com/JasonChu1313/KinectUtil



### The Result:
We can see that the color mismatch reduced a lot in our method case, and we get better result than Kinect API.

By Kinect:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-0deb46f5a8f853d8.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-268787677d9027de.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

By Our Method:

![image.png](http://upload-images.jianshu.io/upload_images/7058214-232596e6da87d9ef.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-8037a9126f9fd8a7.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-8e451cc04663f797.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


  Thanks for reading, if you have any further problems feel free to contact me, this work is done when I work as a research assistant in Hong Kong University Computer Vision & Graphics Lab.


_**Copyright by - Siyu Zhu**_
