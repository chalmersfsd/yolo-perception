/*
 * Copyright (C) 2019   Felix Hörnschemeyer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * This file is there to process the data coming from the image processing of the CNN.
 * The data is then processed to generate birdview coordinates of the cone positions.
 */
#include "lynx_perception.hpp"

//Define intrinsic camera parameters inside matrix (left/right/stereo)
cv::Mat mtxLeft = (cv::Mat_<double>(3, 3) <<
    349.891, 0, 334.352,
    0, 349.891, 187.937,
    0, 0, 1);

cv::Mat mtxRight = (cv::Mat_<double>(3, 3) <<
    350.112, 0, 345.88,
    0, 350.112, 189.891,
    0, 0, 1);

cv::Mat mtxMiddle = (cv::Mat_<double>(3, 3) <<
    350.002, 0, 340.116,
    0, 350.002, 188.914,
    0, 0, 1);

// Declaration of local function
int xyz2xy(cv::Mat Q, cv::Point3f xyz, cv::Point& xy, float radius);

// Function to process 3d points in a frame into 2d coordinates
int xyz2xy(cv::Mat Q, detection xyz, cv::Point &xy, float radius){
  float X = xyz.x_center;
  float Y = xyz.y_center;
  float Z = xyz.z_center;
  float Cx = float(-Q.at<double>(0,3));
  float Cy = float(-Q.at<double>(1,3));
  float f = float(Q.at<double>(2,3));
  float a = float(Q.at<double>(3,2));
  float b = float(Q.at<double>(3,3));
  float d = (f - Z * b ) / ( Z * a);  
  
  // Why does one calculate everything as floats and stores it as an int?
  xy.x = int(X * ( d * a + b ) + Cx);
  xy.y = int(Y * ( d * a + b ) + Cy);
  
  // Probably not necessary in our case
  return int(radius * ( d * a + b ));
}

// Take the cone data and prepare to send out, needs to be called after the network is done with the calculation of a frame
opendlv::cfsdPerception::Cones8 CalculateCone2xy(detection* deCode, size_t numberCones) {
    
    opendlv::cfsdPerception::Cones8 cones; 
    cv::Point xy = {0,0}; 

    if(numberCones > 0)
    {
        xyz2xy(mtxLeft, deCode[0], xy, 0.3f);
        cones.x1(xy.x);
        cones.y1(xy.y);
        cones.class1(deCode[0].classifier);
        cones.confidence1(deCode[0].confidence);
    }
    if(numberCones > 1)
    {   
        xyz2xy(mtxLeft, deCode[1], xy, 0.3f);        
        cones.x2(xy.x);
        cones.y2(xy.y);
        cones.class2(deCode[1].classifier);
        cones.confidence2(deCode[1].confidence);
    }
    if(numberCones > 2)
    {
        xyz2xy(mtxLeft, deCode[2], xy, 0.3f);
        cones.x3(xy.x);
        cones.y3(xy.y);
        cones.class3(deCode[2].classifier);
        cones.confidence3(deCode[2].confidence);
    }
    if(numberCones > 3)
    {   
        xyz2xy(mtxLeft, deCode[3], xy, 0.3f);        
        cones.x4(xy.x);
        cones.y4(xy.y);
        cones.class4(deCode[3].classifier);
        cones.confidence4(deCode[3].confidence);
    }
    if(numberCones > 4)
    {
        xyz2xy(mtxLeft, deCode[4], xy, 0.3f);
        cones.x5(xy.x);
        cones.y5(xy.y);
        cones.class5(deCode[4].classifier);
        cones.confidence5(deCode[4].confidence);
    }
    if(numberCones > 5)
    {   
        xyz2xy(mtxLeft, deCode[5], xy, 0.3f);        
        cones.x6(xy.x);
        cones.y6(xy.y);
        cones.class6(deCode[5].classifier);
        cones.confidence6(deCode[5].confidence);
    }
    if(numberCones > 6)
    {
        xyz2xy(mtxLeft, deCode[6], xy, 0.3f);
        cones.x7(xy.x);
        cones.y7(xy.y);
        cones.class7(deCode[6].classifier);
        cones.confidence7(deCode[6].confidence);
    }
    if(numberCones > 7)
    {   
        xyz2xy(mtxLeft, deCode[7], xy, 0.3f);        
        cones.x8(xy.x);
        cones.y8(xy.y);
        cones.class8(deCode[7].classifier);
        cones.confidence8(deCode[7].confidence);
    }

    return cones;
}