# KinectUtil


This project solves the problem of mismatching between rgb camera and depth camera of Kinect camera. And we can get higher quality point cloud model than Kinect itself. We solve the problem by firstly using both DLT and Zhangzhengyou‘s checkerboard to calibrate the camera, and then applying the calibrated parameters to project and reproject from the image captured by RGB and depth camera.

This project serves as a tutorial for calibrating Kinect camera and tool for project and reproject between different coordinate systems.


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


### The Result:
We can see that the color mismatch reduced a lot in our method case, and we get better result than Kinect API.

By Kinect:
![image.png](http://upload-images.jianshu.io/upload_images/7058214-0deb46f5a8f853d8.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-268787677d9027de.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

By Our Method:

![image.png](http://upload-images.jianshu.io/upload_images/7058214-232596e6da87d9ef.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-8037a9126f9fd8a7.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

![image.png](http://upload-images.jianshu.io/upload_images/7058214-8e451cc04663f797.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


  Thanks for reading, if you have any further problems, please feel free to contact me, this work is done when I worked as a research assistant in Hong Kong University Computer Vision & Graphics Lab.


_**Copyright by - HKU , Siyu Zhu**_
