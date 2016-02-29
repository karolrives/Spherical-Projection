//
//  main.cpp
//  SphericalProjection
//
//  Created by Alejandro Mata Sánchez on 2016-02-27.
//  Copyright © 2016 Amatasan. All rights reserved.
//

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
using namespace cv;
using namespace std;

Mat camera_matrix = (Mat_<double> (3,3,CV_64F) <<
                     4.6177269268364245e+02, 0., 4.0415943610637493e+02, 0.,
                     3.4983097652409509e+02, 2.3940284071090295e+02, 0., 0., 1.);
Mat dist_Coeffs = (Mat_ <double>(5,1) <<
                   -1.7856685671821959e-01, -4.2341319185294768e-02,
                   -8.1590338470856581e-03, -1.1945591085871173e-02,
                   2.2942502481888640e-02 );
Mat x_mapping,y_mapping;


void getSphericalMap(Mat image,Mat &map_x,Mat &map_y,float focal_length){
    float width = image.cols;
    float height = image.rows;
    float xc = 0.5f * width;
    float yc = 0.5f * height;
    float radius = 1;
    for(int j=0; j < image.rows ; j++){
        for(int i=0; i < image.cols ; i++){
            
//            double _x = s * atan2f(i/M_PI, focal_length/M_PI);
//            double _y = s * atan2f(j/M_PI, sqrt(i*i + focal_length*focal_length)/M_PI);
//            
//            double u = s * tan((_x/s)/M_PI);
//            double v = focal_length * tan((_y/s)/M_PI) * (1/cos((_x/s)/M_PI));
//            cout << "X: " << _x << ",Y: " << _y << endl;
//            cout << "U: " << u << ",V: " << v << endl << endl;
            cout << "(i: " << i << ",j: " << j << ")" << endl;
            float tetha =  ((j - xc )  / focal_length);
            float phi = ((i - yc)   / focal_length);
            //cout << "Theta: " << tetha << " Phi: " << phi << endl;
            cout << "Theta(deg): " << tetha*180/M_PI << " Phi(deg): " << phi *180/M_PI << endl;
            float x = 1 * radius * sin( tetha ) * cos ( phi );
            float z = 1 * radius * cos( tetha ) * cos ( phi );
            float y = -1 * radius * sin ( phi  );
            cout << "X: " << x << ",Y: " << y << ",Z: " << z << endl;
            float r = sqrt( x*x + y*y + z*z);
            cout << "Radius: " << r <<endl;
//            float u = atan2(tetha, phi)/M_PI_2;
//            while (u>=1.0) u -= 1.0;
//            while (u<0) u += 1.0;
//            phi /= sqrt(tetha*tetha+phi*phi);
//            float v = asin(phi)/M_PI;
            float u = ( x / z ) * focal_length + xc ;
            float v = ( y / z ) * focal_length + yc ;
            map_x.at<float>(j,i) = u;
            map_y.at<float>(j,i) = v;
            cout << "U: " << u << ",V: " << v << endl << endl;
            
        }
    }
    
}

int main(int argc, const char * argv[]) {
    Mat src,dst;
    Mat undistorted_image,temp;
    Mat affineMatrix;
    Mat big_screen(1280,720,CV_32F);
    string window="Spherical Projection";
    VideoCapture cam;
    
    int cam_id = atoi(argv[1]);
    cout << "Openning cam: " << cam_id << endl;
    cam.open(cam_id);
    namedWindow(window,WINDOW_OPENGL);
    
    if(!cam.isOpened()){
        return EXIT_FAILURE;
    }
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    cam >> src;
    
    
    float scale = 0.75f;
    bool undistort = false;
    x_mapping.create( src.size(), CV_32FC1 );
    y_mapping.create( src.size(), CV_32FC1 );
    dst = Mat(src.size(),src.type());
    float focal_length = 100 ;
    getSphericalMap(src,x_mapping,y_mapping,focal_length);
    float theta = 90,phi =90, rho =90  ;


    while(true){
        cam >> src;
        remap(src, dst, y_mapping, x_mapping, INTER_CUBIC);
        Size image_size = src.size();
//        Mat A1 = (Mat_<double>(4,3) <<
//                  1, 0, -w/2,
//                  0, 1, -h/2,
//                  0, 0,    0,
//                  0, 0,    1);
//        Mat rotation_matrix_Z = (cv::Mat_ <double> (4,4,CV_64F) <<
//                                   cos(theta*M_PI/180),-1*sin(theta*M_PI/180),0,0,
//                                   sin(theta*M_PI/180),cos(theta*M_PI/180),0,0,
//                                   0,0,1,0,
//                                   0,0,0,1);
//        Mat rotation_matrix_Y = (cv::Mat_ <double> (4,4,CV_64F) <<
//                                 cos(rho*M_PI/180),0,sin(rho*M_PI/180),0,
//                                 0,1,0,0,
//                                 sin(rho*M_PI/180),0,cos(rho*M_PI/180),0,
//                                 0,0,0,1);
//        Mat rotation_matrix_X = (cv::Mat_ <double> (4,4,CV_64F) <<
//                                 1,0,0,0,
//                                 0,cos(phi*M_PI/180),-1*sin(phi*M_PI/180),0,
//                                 0,sin(phi*M_PI/180),cos(phi*M_PI/180),0,
//                                 0,0,0,1);
//        Mat rotation_matrix = rotation_matrix_Z;

        // Rotation matrices around the X, Y, and Z axis
//        Mat RX = (Mat_<double>(4, 4) <<
//                  1,          0,           0, 0,
//                  0, cos(theta*M_PI/180), -sin(theta*M_PI/180), 0,
//                  0, sin(theta*M_PI/180),  cos(theta*M_PI/180), 0,
//                  0,          0,           0, 1);
//        Mat RY = (Mat_<double>(4, 4) <<
//                  cos(rho*M_PI/180), 0, -sin(rho*M_PI/180), 0,
//                  0, 1,          0, 0,
//                  sin(rho*M_PI/180), 0,  cos(rho*M_PI/180), 0,
//                  0, 0,          0, 1);
//        Mat RZ = (Mat_<double>(4, 4) <<
//                  cos(phi*M_PI/180), -sin(phi*M_PI/180), 0, 0,
//                  sin(phi*M_PI/180),  cos(phi*M_PI/180), 0, 0,
//                  0,          0,           1, 0,
//                  0,          0,           0, 1);
//        Mat T = (Mat_<double>(4, 4) <<
//                 1, 0, 0, 0,
//                 0, 1, 0, 0,
//                 0, 0, 1, 500,
//                 0, 0, 0, 1);
//        
//        Mat A2 = (Mat_<double>(3,4) <<
//                  f, 0, w/2, 0,
//                  0, f, h/2, 0,
//                  0, 0,   1, 0);
//        Mat R = RX * RY * RZ;
//        Mat trans = A2 * (T * (R * A1));
//        warpPerspective(src, dst, trans, src.size(),INTER_LANCZOS4);
//      if(undistort){
//            Size image_size = src.size();
//            affineMatrix =(cv::Mat_ <double> (2,3,CV_64F) << scale,0,0,0,scale,0);
//            //cv::warpPerspective(image,tmp,camera_matrix,img_size,INTER_LINEAR,BORDER_CONSTANT);
//            cv::Point2f image_center(dst.cols *  0.5, dst.rows * 0.5);
//            cv::Point2f scaled_center(dst.cols * scale *  0.5, dst.rows * scale * 0.5);
//            affineMatrix.at<double>(0,2) += image_center.x - scaled_center.x;
//            affineMatrix.at<double>(1,2) += image_center.y - scaled_center.y;
//            cv::warpAffine(dst,temp, affineMatrix,image_size);
//            cv::undistort(temp,undistorted_image,camera_matrix,dist_Coeffs*scale);
//            Size scaled_image_size = undistorted_image.size();
//            std::cout << "Size: " << scaled_image_size.width << "," << scaled_image_size.height << std::endl;
//            std::cout << "Center: " << image_center.x << "," << image_center.y << std::endl;
//            std::cout << "Scale: " << scale << std::endl;
//            std::cout << std::endl;
//        }
//        else{
//            src.copyTo(undistorted_image);
//        }
        imshow(window, dst );
        
        char key = waitKey(1);
        switch(key){
            case '1':
                scale += 0.01;
                std::cout << "Scale: " << scale << std::endl;
                break;
            case 'q':
                scale -= 0.01;
                std::cout << "Scale: " << scale << std::endl;
                break;
            case '2':
                theta += 10;
                std::cout << "theta: " << theta << std::endl;
                break;
            case 'w':
                theta -= 10;
                std::cout << "theta: " << theta << std::endl;
                break;
            case '3':
                rho += 10;
                std::cout << "rho: " << rho << std::endl;
                break;
            case 'e':
                rho -= 10;
                std::cout << "rho: " << rho << std::endl;
                break;
            case '4':
                phi += 10;
                std::cout << "phi: " << phi << std::endl;
                break;
            case 'r':
                phi -= 10;
                std::cout << "phi: " << phi << std::endl;
                break;
            case 'u':
                if(!undistort)
                    undistort=true;
                else
                    undistort=false;
                break;
        }
        if (key == 27) {
            break;
        }


    }
    src.release();
    dst.release();
    return 0;
}
