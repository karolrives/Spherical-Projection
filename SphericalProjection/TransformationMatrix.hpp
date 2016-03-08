//
//  TransformationMatrix.hpp
//  test
//
//  Created by A Carolina Figueroa Rives on 2016-03-02.
//  Copyright © 2016 A Carolina Figueroa Rives. All rights reserved.
//

#ifndef TransformationMatrix_hpp
#define TransformationMatrix_hpp
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::detail;


class Transformation {

private:
    Mat image;
    Mat R, RX, RY, RZ;  // Rotation Matrices
    Mat T;              // Translation Matrix
    Mat C1, C2;         // 2D to 3D & vice
    Mat Affine;
    Mat CameraMatrix;
    double alpha;       // x
    double beta;        // y
    double gamma;       // z
    float scale;

    double dx, dy, dz;  // distances from cam
    int f;              // focal lenght

public:
    double width, height;

    Transformation(Mat image, double alpha, double beta, double gamma,
                   double dx, double dy, double dz, int f);

    Mat getRotationMatrix();
    Mat getTranslationMatrix();
    Mat getTransformationMatrix();
    double getAngle(char axis);
    Mat getAffineMatrix(float scale);
    Mat getCameraMatrix();

    void setSphericalTMatrix(cv::Mat &map_x, cv::Mat &map_y);
    void setAngle(char axis, double value);
    void setFocalLength(double value);
    void setRotMatOnX();
    void setRotMatOnY();
    void setRotMatOnZ();
    void setTranslationMatrix();
    void setDistancePoints(double x, double y, double z);

    //void setAffineMatrix(float scale);

    //cv::Mat set2dTo3dMatrix();
    //cv::Mat set3dTo2dMatrix();


};
#endif /* TransformationMatrix_hpp */
