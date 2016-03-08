//
//  main.cpp
//  SphericalProjection
//
//  Created by Alejandro Mata Sánchez and Karol Rives on 2016-02-27.
//  Copyright © 2016 Amatasan. All rights reserved.
//

#include "TransformationMatrix.hpp"


Mat camera_matrix;
Mat dist_Coeffs = (Mat_ <double>(5,1) <<
                   -3.4736267613379468e-01, 8.0321530340705952e-02,
                          -4.0753376664399918e-03, -1.9895964012628582e-04,
                          2.1732257892117041e-02);
Mat x_mapping,y_mapping;


int main(int argc, const char * argv[]) {
    
    Mat src, dst, trans;
    Mat undistorted_image,temp,remapped;
    Mat affineMatrix;
    
    float scale = 0.75f;
    bool undistort = false;
    bool needs_remap = false;
    float theta = 0, rho = 0, phi = 0 ;
    float offset_x = 0.,offset_y=0.;
    float focal_length = 250 ;
    
    string window="Spherical Projection";
    VideoCapture cam;

    cout << "Openning cam: " << argv[1] << endl;
    cam.open(atoi(argv[1]));
    
    if(!cam.isOpened()){
        cerr << "Couldn't Open the video: " << argv[1] << endl;
        return -1;
    }
    namedWindow(window,WINDOW_FULLSCREEN);
    cam >> src;

    x_mapping.create( src.size(), CV_32FC1 );
    y_mapping.create( src.size(), CV_32FC1 );
    dst.create(src.size(), src.type());
    //dst = Mat(src.size(),src.type());
    
    Transformation T_(src,theta,rho,phi,offset_x,offset_y,focal_length,focal_length);
    camera_matrix = T_.getCameraMatrix();
    T_.setSphericalTMatrix(x_mapping, y_mapping);
   
    while(true){
    
        cam >> src;

        if(src.empty()){
            break;
          }
    
        trans = T_.getTransformationMatrix();
       

        if(undistort && src.size().width != 0){
            
            affineMatrix = T_.getAffineMatrix(scale);
            cv::warpAffine(src,temp, affineMatrix,src.size());
            cv::undistort(temp,undistorted_image,camera_matrix,dist_Coeffs);

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
                T_.setAngle('x', theta);
                std::cout << "theta: " << theta << std::endl;
                cout << "Alpha: " << T_.getAngle('x') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case 'w':
                theta -= 10;
                T_.setAngle('x', theta);
                std::cout << "theta: " << theta << std::endl;
                cout << "Alpha: " << T_.getAngle('x') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case '3':
                rho += 10;
                T_.setAngle('y', rho);
                std::cout << "rho: " << rho << std::endl;
                cout << "Beta: " << T_.getAngle('y') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case 'e':
                rho -= 10;
                T_.setAngle('y', rho);
                std::cout << "rho: " << rho << std::endl;
                cout << "Beta: " << T_.getAngle('y') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case '4':
                phi += 10;
                T_.setAngle('z', phi);
                std::cout << "phi: " << phi << std::endl;
                cout << "Gamma: " << T_.getAngle('z') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case 'r':
                phi -= 10;
                T_.setAngle('z', phi);
                std::cout << "phi: " << phi << std::endl;
                cout << "Gamma: " << T_.getAngle('z') << endl;
                cout << T_.getRotationMatrix() << endl;
                break;
            case 'f':
                focal_length += 10;
                T_.setFocalLength(focal_length);
                std::cout << "Focal Length: " << focal_length << std::endl;
                T_.setSphericalTMatrix(x_mapping, y_mapping);
                break;
            case 'd':
                focal_length -= 10;
                T_.setFocalLength(focal_length);
                std::cout << "Focal Length: " << focal_length << std::endl;
                T_.setSphericalTMatrix(x_mapping, y_mapping);
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

