//
//  TransformationMatrix.cpp
//  test
//
//  Created by A Carolina Figueroa Rives on 2016-03-02.
//  Copyright © 2016 A Carolina Figueroa Rives. All rights reserved.
//

#include "TransformationMatrix.hpp"

Transformation::Transformation(cv::Mat image, double alpha, double beta, double gamma,
                   double dx, double dy, double dz, int f){
        
    this->image = Mat(image.size(),image.type());
    this->image = image;
    width = image.cols;
    height = image.rows;
    this->alpha = (alpha) * CV_PI/180;  //(alpha - 90) * CV_PI/180;
    this->beta =  (beta) * CV_PI/180;
    this->gamma = (gamma) * CV_PI/180;
    this->dx = dx;
    this->dy = dy;
    this->dz = dz;
    this->f = f;
    
    CameraMatrix = (Mat_<double> (3,3,CV_64F) <<
                     3.4798561303686398e+02, 0., width/2,
                     0., 3.4931440793173061e+02, height/2,
                     0., 0., 1.  );
    
    C1 = (cv::Mat_<double>(4, 3) <<
          1, 0, -width/2,
          0, 1, -height/2,
          0, 0, 0,
          0, 0, 1);
    C2 = (cv::Mat_<double>(3, 4) <<
          f, 0, width/2,0,
          0, f, height/2,0,
          0, 0, 1,0);
    
    setRotMatOnX();
    setRotMatOnY();
    setRotMatOnZ();
    setTranslationMatrix();
    
}

Mat Transformation::getRotationMatrix(){
        
    R = RX * RY * RZ;
    return R;

}

Mat Transformation::getTranslationMatrix(){
    
    return T;
}

Mat Transformation::getTransformationMatrix(){
    
    Mat Tr = C2 *( getTranslationMatrix() * getRotationMatrix() * C1);
    return Tr;
    
}

double Transformation::getAngle(char axis){
    
    if (axis == 'x')
        return alpha;
    else if (axis == 'y')
        return beta;
    else if (axis == 'z')
        return gamma;
    else
        return 0;
    
}

Mat Transformation::getCameraMatrix(){
    
    return CameraMatrix;
}


void Transformation::setSphericalTMatrix(cv::Mat &map_x, cv::Mat &map_y){
    
    float xc = 0.5f * width;
    float yc = 0.5f * height;
    
    cout << "Calculating Spherical remapping" << endl;
    for(int j = 0; j < height; j++){
        for(int i = 0; i < width; i++){
            
            float tetha =  ((i - xc )  / f);
            float phi = ((j - yc)   / f);
            
            float x = sin( tetha ) * cos ( phi );
            float z = cos( tetha ) * cos ( phi );
            float y = sin ( phi  );
            
            float u = ( x / z ) * f + xc ;
            float v = (y / z ) * f + yc ;
            
            map_x.at<float>(j,i) = u;
            map_y.at<float>(j,i) = v;
            //cout << ".";
        }
    }
    cout << "Done" << endl;

}

void Transformation::setAngle(char axis, double value){
    
    if (axis == 'x'){
        alpha = (value) * CV_PI/180;
        setRotMatOnX();
    }
    else if (axis == 'y'){
        beta = (value) * CV_PI/180;
        setRotMatOnY();
    }
    else if (axis == 'z'){
        gamma = (value) * CV_PI/180;
        setRotMatOnZ();
    }
    
}

void Transformation::setFocalLength(double value){
    
    f = value;
    C2 = (cv::Mat_<double>(3, 4) <<     // I shouldnt do this :/
          f, 0, width/2,0,
          0, f, height/2,0,
          0, 0, 1,0);
}

void Transformation::setRotMatOnX(){
    
    RX = (Mat_ <double> (4,4) <<
          1,0,0,0,
          0,cos(alpha),-sin(alpha),0,
          0,sin(alpha),cos(alpha),0,
          0,0,0,1);
}

void Transformation::setRotMatOnY(){
    
    RY = (Mat_<double>(4, 4) <<
          cos(beta), 0, -sin(beta), 0,
          0, 1,          0, 0,
          sin(beta), 0,  cos(beta), 0,
          0, 0,          0, 1);
}

void Transformation::setRotMatOnZ(){
    
    RZ = (Mat_<double>(4, 4) <<
          cos(gamma), -sin(gamma), 0, 0,
          sin(gamma),  cos(gamma), 0, 0,
          0,          0,           1, 0,
          0,          0,           0, 1);
}

void Transformation::setDistancePoints(double x, double y, double z){
    
    dx = x;
    dy = y;
    dz = z;
    setTranslationMatrix();
    
}

void Transformation::setTranslationMatrix(){
    
    T = (cv::Mat_<double>(4, 4) <<
         1, 0, 0, dx,
         0, 1, 0, dy,
         0, 0, 1, dz,
         0, 0, 0, 1);
}

Mat Transformation::getAffineMatrix(float scale){
    
    this->scale = scale;
    double xc = width/2;
    double yc = height/2;
    
    Affine =(cv::Mat_ <double> (2,3,CV_64F) <<
             scale, 0, xc*(1- scale),
             0, scale, yc*(1- scale)
             );
    /*Point2f image_center(width *  0.5, height * 0.5);
    Point2f scaled_center(width * scale *  0.5, height * scale * 0.5);
    Affine.at<double>(0,2) += image_center.x - scaled_center.x;
    Affine.at<double>(1,2) += image_center.y - scaled_center.y;*/
    return Affine;
    

}



