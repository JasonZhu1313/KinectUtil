//
//  directlineartransform.cpp
//  KinectTracking
//
//  Created by 思宇朱 on 2017/10/29.
//  Copyright © 2017年 思宇朱. All rights reserved.
//
#include "directlineartransform.hpp"
float DLT::calculateError(vector<Point2d> projectedPoint){
    float distant=0;
    if(projectedPoint.size()==this->imagePoints.size()){
        for(int i=0;i<projectedPoint.size();i++){
            float xpro=(float)projectedPoint[i].x;
            float ypro=(float)projectedPoint[i].y;
            float ximage=(float)this->imagePoints[i].x;
            float yimage=(float)this->imagePoints[i].y;
            
            float sqrtvar =sqrt ( (xpro-ximage)* (xpro-ximage)+ (ypro-yimage)* (ypro-yimage));
            distant += sqrtvar;
            
        }
    }
    cout<<"mean error (pixel) = "<< distant/projectedPoint.size() <<endl;
    return distant;
}
int DLT::generatePointCloud(Mat colorMat,vector<Point3d> cameraSpacePoints,vector<Point2d> refinedPoint,std::string outputFilename){
    std::ofstream fileout(outputFilename);
    if(!fileout.is_open()){
        cout<<"error when write to the file"<<endl;
        return -1;
    }
    fileout << "COFF" << std::endl;
    fileout << cameraSpacePoints.size() << " 0 0" << std::endl;
    for (int i = 0; i < cameraSpacePoints.size(); i++)
    {
        Point2d mappingRGB = refinedPoint[i];
        fileout << cameraSpacePoints[i].x << " " << cameraSpacePoints[i].y << " " << cameraSpacePoints[i].z << " " << (int)colorMat.at<Vec3b>(mappingRGB.x,mappingRGB.y).val[0]<< " " << (int)colorMat.at<Vec3b>(mappingRGB.x,mappingRGB.y).val[1]<< " " << (int)colorMat.at<Vec3b>(mappingRGB.x,mappingRGB.y).val[2]<< "255" << std::endl;
    }
    fileout.close();
    return 1;
}
vector<Point2d> DLT::reprojectToColorSpace(vector<Point3d> cameraSpacePoints){
    vector<Point2d> colorspacePoint;
    if(!this->p.empty()&&cameraSpacePoints.size()>0){
        for(int i=0;i<cameraSpacePoints.size();i++){
            Point3d p3d=cameraSpacePoints[i];
            double x = p3d.x;
            double y = p3d.y;
            double z = p3d.z;
            Mat p3dmat = (Mat_<float>(1, 4) << x, y, z, 1);
            double projectedx = p3dmat.dot(pline1)/p3dmat.dot(pline3);
            double projectedy = p3dmat.dot(pline2)/p3dmat.dot(pline3);
            colorspacePoint.push_back(Point2d(projectedx,projectedy));
        }
    }
    return colorspacePoint;
}
Mat DLT::getp(){
    return this->p;
}
int DLT::readDataFromFile(std::string imagePointsPath, std::string objectPointsPath){
    int objectPointCounts=0;
    std::ifstream filereader;
    filereader.open(imagePointsPath);
    if(!filereader.is_open()){
        cout<<"error happen when open "<<imagePointsPath<<endl;
        return -1;
    }else{
        int i=0;
        while(!filereader.eof()){
            double x;
            double y;
                filereader >> x >> y;
                cout<<x<<","<<y<<endl;
                imagePoints.push_back(Point2d(x,y));
                imageCount++;
            
        }
    }
    filereader.close();
    filereader.open(objectPointsPath);
    if(!filereader.is_open()){
        cout<<"error happen when open "<<imagePointsPath<<endl;
        return -1;
    }else{
        int i=0;
        while(!filereader.eof()){
            double x;
            double y;
            double z;
           
                filereader >> x >> y >> z ;
                cout<<x<<","<<y<<","<<z<<endl;
                objectPoints.push_back(Point3d(x,y,z));
                objectPointCounts++;
            
        }
    }
    if(imageCount!=objectPointCounts){
        cout<<"error in count"<<endl;
        return -1;
    }
    return 1;
}

VectorXf DLT::initEigen(){
    if(imageCount==0){
        cout<<"not image point to process"<<endl;
    }
    MatrixXf A = MatrixXf::Zero(2 * imageCount, 11);
    VectorXf b = VectorXf::Zero(2 * imageCount);
    for (int i = 0; i < imageCount; i++){
       
        A(i * 2, 0) = objectPoints[i].y;
        A(i * 2, 1) = objectPoints[i].z;
        A(i * 2, 2) = 1;
        A(i * 2, 7) = -objectPoints[i].x*imagePoints[i].x;
        A(i * 2, 8) = -objectPoints[i].y*imagePoints[i].x;
        A(i * 2, 9) = -objectPoints[i].z*imagePoints[i].x;
        A(i * 2, 10) =-imagePoints[i].x;
        
        A(i * 2 + 1, 3) = objectPoints[i].x;
        A(i * 2 + 1, 4) = objectPoints[i].y;
        A(i * 2 + 1, 5) = objectPoints[i].z;
        A(i * 2 + 1, 6) = 1;
        A(i * 2 + 1, 7) = -objectPoints[i].x*imagePoints[i].y;
        A(i * 2 + 1, 8) = -objectPoints[i].y*imagePoints[i].y;
        A(i * 2 + 1, 9) = -objectPoints[i].z*imagePoints[i].y;
        A(i * 2 + 1, 10) = -imagePoints[i].y;
        b(i * 2) = objectPoints[i].x;
    }
    VectorXf solution = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
    for(int i=0;i<solution.size();i++){
        cout<<solution[i]<<endl;
    }
    
    return solution;
}

int DLT::init(){
    if(imageCount==0){
        cout<<"not image point to process"<<endl;
        return -1;
    }
    p.create(3,4,CV_32FC1);
    A.create(2*imageCount,11,CV_32FC1);
    B.create(2*imageCount,1,CV_32FC1);
    paramMat.create(11,1,CV_32FC1);
    pline1.create(1, 4, CV_32FC1);
    pline2.create(1, 4, CV_32FC1);
    pline3.create(1, 4, CV_32FC1);
    for(int i=0;i<imageCount;i++)
    {
        //axT
        A.at<float>(i*2,0)=objectPoints[i].y;
        A.at<float>(i*2,1)=objectPoints[i].z;
        A.at<float>(i*2,2)=1;
        A.at<float>(i*2,3)=0;
        A.at<float>(i*2,4)=0;
        A.at<float>(i*2,5)=0;
        A.at<float>(i*2,6)=0;
        A.at<float>(i*2,7)=-objectPoints[i].x*imagePoints[i].x;
        A.at<float>(i*2,8)=-objectPoints[i].y*imagePoints[i].x;
        A.at<float>(i*2,9)=-objectPoints[i].z*imagePoints[i].x;
        A.at<float>(i*2,10)=-imagePoints[i].x;
     
        
        //ayT
        A.at<float>(i*2+1,0)=0;
        A.at<float>(i*2+1,1)=0;
        A.at<float>(i*2+1,2)=0;
        A.at<float>(i*2+1,3)=objectPoints[i].x;
        A.at<float>(i*2+1,4)=objectPoints[i].y;
        A.at<float>(i*2+1,5)=objectPoints[i].z;
        A.at<float>(i*2+1,6)=1;
        A.at<float>(i*2+1,7)=-objectPoints[i].x*imagePoints[i].y;
        A.at<float>(i*2+1,8)=-objectPoints[i].y*imagePoints[i].y;
        A.at<float>(i*2+1,9)=-objectPoints[i].z*imagePoints[i].y;
        A.at<float>(i*2+1,10)=-imagePoints[i].y;

//        //axT
//        A.at<float>(i,0)=objectPoints[i/2].x;
//        A.at<float>(i,1)=objectPoints[i/2].y;
//        A.at<float>(i,2)=objectPoints[i/2].z;
//        A.at<float>(i,3)=1;
//        A.at<float>(i,4)=0;
//        A.at<float>(i,5)=0;
//        A.at<float>(i,6)=0;
//        A.at<float>(i,7)=0;
//        A.at<float>(i,8)=-objectPoints[i/2].x*imagePoints[i/2].x;
//        A.at<float>(i,9)=-objectPoints[i/2].y*imagePoints[i/2].x;
//        A.at<float>(i,10)=-objectPoints[i/2].z*imagePoints[i/2].x;
//        //ayT
//        A.at<float>(i+1,0)=0;
//        A.at<float>(i+1,1)=0;
//        A.at<float>(i+1,2)=0;
//        A.at<float>(i+1,3)=0;
//        A.at<float>(i+1,4)=objectPoints[i/2].x;
//        A.at<float>(i+1,5)=objectPoints[i/2].y;
//        A.at<float>(i+1,6)=objectPoints[i/2].z;
//        A.at<float>(i+1,7)=1;
//        A.at<float>(i+1,8)=-objectPoints[i/2].x*imagePoints[i/2].y;
//        A.at<float>(i+1,9)=-objectPoints[i/2].y*imagePoints[i/2].y;
//        A.at<float>(i+1,10)=-objectPoints[i/2].z*imagePoints[i/2].y;
    }
    for(int i=0;i<imageCount;i++)
    {
        B.at<float>(i*2,0)=-objectPoints[i].x;
        B.at<float>(i*2+1,0)=0;
    }
    return 1;
}

int DLT::calculateParam(){
    for(int i=0;i<p.rows;i++)
    {
        for(int j=0;j<p.cols;j++)
        {
            p.at<float>(i,j)=0.0f;
        }
    }
    for(int i=0;i<paramMat.rows;i++)
    {
        for(int j=0;j<paramMat.cols;j++)
        {
            paramMat.at<float>(i,j)=0.0f;
        }
    }
    if(!solve(A,B,paramMat,CV_SVD)){
        cout<<"svd error"<<endl;
        return -1;
    }
    for(int i=0;i<paramMat.rows;i++){
        cout<< "parammat" << paramMat.at<float>(i,0)<<endl;
        
    }
//    float m3_4=1.0;
//    for(int i=0;i<p.rows;i++)
//    {
//        for(int j=0;j<p.cols;j++)
//        {
//            if(i==2&&j==3)
//            {
//                p.at<float>(i,j)=m3_4;
//            }
//            else
//            {
//                p.at<float>(i,j)=m3_4*paramMat.at<float>(i*4+j);
//            }
//        }
//    }
    float m1_1=1.0;
    for(int i=0;i<p.rows;i++)
    {
        for(int j=0;j<p.cols;j++)
        {
            if(i==0&&j==0)
            {
                p.at<float>(i,j)=m1_1;
            }
            else
            {
                p.at<float>(i,j)=paramMat.at<float>(i*4+j-1);
            }
        }
    }
    //init the param
    for(int i=0;i<4;i++){
        pline1.at<float>(0,i)=p.at<float>(0,i);
        //cout<<"********"<<p.at<float>(0,i)<<endl;
        pline2.at<float>(0,i)=p.at<float>(1,i);
        //cout<<"++++++++"<<p.at<float>(1,i)<<endl;
        pline3.at<float>(0,i)=p.at<float>(2,i);
        //cout<<"--------"<<p.at<float>(2,i)<<endl;
    }
    return 1;
}




