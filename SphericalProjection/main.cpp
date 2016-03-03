//
//  main.cpp
//  SphericalProjection
//
//  Created by Alejandro Mata Sánchez and Karol Rives on 2016-02-27.
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

Mat camera_matrix;
Mat dist_Coeffs = (Mat_ <double>(5,1) <<
                   -3.4736267613379468e-01, 8.0321530340705952e-02,
                          -4.0753376664399918e-03, -1.9895964012628582e-04,
                          2.1732257892117041e-02);
Mat x_mapping,y_mapping;


void getSphericalMap(Mat image,Mat &map_x,Mat &map_y,float focal_length){
  float width = image.cols;
  float height = image.rows;
  float xc = 0.5f * width;
  float yc = 0.5f * height;

  cout << "Calculating Spherical remapping" << endl;
  for(int j=0; j < image.rows ; j++){
      for(int i=0; i < image.cols ; i++){
          //cout << "(i: " << i << ",j: " << j << ")" << endl;
          float tetha =  ((i - xc )  / focal_length);
          float phi = ((j - yc)   / focal_length);
          //cout << "Theta: " << tetha << " Phi: " << phi << endl;
          //cout << "Theta(deg): " << tetha*180/M_PI << " Phi(deg): " << phi *180/M_PI << endl;
          float x = sin( tetha ) * cos ( phi );
          float z = cos( tetha ) * cos ( phi );
          float y = sin ( phi  );
          //cout << "X: " << x << ",Y: " << y << ",Z: " << z << endl;
          //float r = sqrt( x*x + y*y + z*z);
          //cout << "Radius: " << r <<endl;
          //cout << "WidthxHeight:" << width << "x" << height <<endl;
          //cout << "Xc,Yc:" << xc << "x" << yc <<endl <<endl;
          float u = ( x / z ) * focal_length + xc ;
          float v = (y / z ) * focal_length + yc ;
          map_x.at<float>(j,i) = u;
          map_y.at<float>(j,i) = v;
          cout << ".";
      }
  }
  cout << "Done" << endl;

}

int main(int argc, const char * argv[]) {
    Mat src,dst;
    Mat undistorted_image,temp,remapped;
    Mat affineMatrix;
    //Mat big_screen(1280,720,CV_32F);
    string window="Spherical Projection";
    VideoCapture cam;

    cout << "Openning cam: " << argv[1] << endl;
    cam.open(argv[1]);
    if(!cam.isOpened()){
            cam.open(atoi(argv[1]));
    }
    if (!cam.isOpened()) {
            cerr << "Couldn't Open the video: " << argv[1] << endl;
            return -1;
    }
    namedWindow(window,WINDOW_FULLSCREEN);
    cam >> src;


    float scale = 0.75f;
    bool undistort = false,needs_remap = false;
    x_mapping.create( src.size(), CV_32FC1 );
    y_mapping.create( src.size(), CV_32FC1 );
    dst = Mat(src.size(),src.type());
    float focal_length = 250 ;
    getSphericalMap(src,x_mapping,y_mapping,focal_length);
    float theta = 0, rho =0, phi =0 ;
    float offset_x = 0.,offset_y=0.;



    while(true){
        cam >> src;

        double width = (double)src.cols;
        double height = (double)src.rows;
        if(src.empty()){
            break;
          }
        Mat A1 = (Mat_<double>(4,3) <<
                  1, 0, -width/2,
                  0, 1, -height/2,
                  0, 0,    0,
                  0, 0,    1);
        double alpha = (theta )*CV_PI/180.;
        double beta = (rho )*CV_PI/180.;
        double gamma = (phi )*CV_PI/180.;

        // Rotation matrices around the X, Y, and Z axis
        Mat RX = (Mat_<double>(4, 4) <<
                  1,          0,           0, 0,
                  0, cos(alpha), -1*sin(alpha), 0,
                  0, sin(alpha),  cos(alpha), 0,
                  0,          0,           0, 1);
        Mat RY = (Mat_<double>(4, 4) <<
                  cos(beta), 0, -1*sin(beta), 0,
                  0,         1,          0, 0,
                  sin(beta), 0,  cos(beta), 0,
                  0,         0,  0,         1);
        Mat RZ = (Mat_<double>(4, 4) <<
                  cos(gamma), -1*sin(gamma), 0, 0,
                  sin(gamma),  cos(gamma), 0, 0,
                  0,          0,           1, 0,
                  0,          0,           0, 1);
        Mat T = (Mat_<double>(4, 4) <<
                 1, 0, 0, offset_x,
                 0, 1, 0, offset_y,
                 0, 0, 1, focal_length,
                 0, 0, 0, 1);

        Mat A2 = (Mat_<double>(3,4) <<
                  focal_length, 0, width/2, 0,
                  0, focal_length, height/2, 0,
                  0, 0, 1, 0);
        Mat R = RX * RY * RZ;
        Mat trans = A2 * (T * (R * A1));


      if(undistort){
            Size image_size = src.size();
            if(image_size.width == 0) break;
            affineMatrix =(cv::Mat_ <double> (2,3,CV_64F) << scale,0,0,0,scale,0);
            //cv::warpPerspective(image,tmp,camera_matrix,img_size,INTER_LINEAR,BORDER_CONSTANT);
            cv::Point2f image_center(src.cols *  0.5, src.rows * 0.5);
            cv::Point2f scaled_center(src.cols * scale *  0.5, src.rows * scale * 0.5);
            affineMatrix.at<double>(0,2) += image_center.x - scaled_center.x;
            affineMatrix.at<double>(1,2) += image_center.y - scaled_center.y;
            cv::warpAffine(src,temp, affineMatrix,image_size);
            camera_matrix = (Mat_<double> (3,3,CV_64F) <<
                                 3.4798561303686398e+02, 0., src.cols/2,
                                         0., 3.4931440793173061e+02, src.rows/2,
                                         0., 0., 1.  );
            cv::undistort(temp,undistorted_image,camera_matrix,dist_Coeffs);
            //Size scaled_image_size = undistorted_image.size();
        }
        else{
            src.copyTo(undistorted_image);
        }
        if(needs_remap){
            remap(undistorted_image, remapped, x_mapping, y_mapping, INTER_CUBIC);
        }
        else{
            undistorted_image.copyTo(remapped);
        }
        warpPerspective(remapped, dst, trans, src.size(),INTER_LANCZOS4);
        imshow(window, dst);

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
                cout << "Alpha: " << alpha << endl;
                cout << R << endl;
                break;
            case 'w':
                theta -= 10;
                std::cout << "theta: " << theta << std::endl;
                cout << "Alpha: " << beta << endl;
                cout << R << endl;
                break;
            case '3':
                rho += 10;
                std::cout << "rho: " << rho << std::endl;
                cout << "Beta: " << beta << endl;
                cout << R << endl;
                break;
            case 'e':
                rho -= 10;
                std::cout << "rho: " << rho << std::endl;
                cout << "Beta: " << beta << endl;
                cout << R << endl;
                break;
            case '4':
                phi += 10;
                std::cout << "phi: " << phi << std::endl;
                cout << "Gamma: " << gamma << endl;
                cout << R << endl;
                break;
            case 'r':
                phi -= 10;
                std::cout << "phi: " << phi << std::endl;
                cout << "Gamma: " << gamma << endl;
                cout << R << endl;
                break;
            case 'f':
                focal_length += 10;
                std::cout << "Focal Length: " << focal_length << std::endl;
                getSphericalMap(src,x_mapping,y_mapping,focal_length);
                break;
            case 'd':
                focal_length -= 10;
                std::cout << "Focal Length: " << focal_length << std::endl;
                getSphericalMap(src,x_mapping,y_mapping,focal_length);
                break;
            case 'p':
              if(!needs_remap)
                  needs_remap=true;
              else
                  needs_remap=false;
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
    return 0;
}

