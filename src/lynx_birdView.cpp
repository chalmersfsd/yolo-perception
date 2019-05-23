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
void CalculateCone2xy(detection DeCode[8]){
    
    cv::Point xy = {0,0}; 
    int Number = DeCode[0].num;

    if(Number > 0)
    {
        xyz2xy(mtxLeft, DeCode[0], xy, 0.3f);
        GlobalConeData.x1 = xy.x;
        GlobalConeData.y1 = xy.y;
        GlobalConeData.class1 = DeCode[0].classifier;
        GlobalConeData.confidence1 = DeCode[0].confidence;
    }
    if(Number > 1)
    {   
        xyz2xy(mtxLeft, DeCode[1], xy, 0.3f);        
        GlobalConeData.x2 = xy.x;
        GlobalConeData.y2 = xy.y;
        GlobalConeData.class2 = DeCode[1].classifier;
        GlobalConeData.confidence2 = DeCode[1].confidence;
    }
    if(Number > 2)
    {
        xyz2xy(mtxLeft, DeCode[2], xy, 0.3f);
        GlobalConeData.x3 = xy.x;
        GlobalConeData.y3 = xy.y;
        GlobalConeData.class3 = DeCode[2].classifier;
        GlobalConeData.confidence3 = DeCode[2].confidence;
    }
    if(Number > 3)
    {   
        xyz2xy(mtxLeft, DeCode[3], xy, 0.3f);        
        GlobalConeData.x4 = xy.x;
        GlobalConeData.y4 = xy.y;
        GlobalConeData.class4 = DeCode[3].classifier;
        GlobalConeData.confidence4 = DeCode[3].confidence;
    }
    if(Number > 4)
    {
        xyz2xy(mtxLeft, DeCode[4], xy, 0.3f);
        GlobalConeData.x5 = xy.x;
        GlobalConeData.y5 = xy.y;
        GlobalConeData.class5 = DeCode[4].classifier;
        GlobalConeData.confidence5 = DeCode[4].confidence;
    }
    if(Number > 5)
    {   
        xyz2xy(mtxLeft, DeCode[5], xy, 0.3f);        
        GlobalConeData.x6 = xy.x;
        GlobalConeData.y6 = xy.y;
        GlobalConeData.class6 = DeCode[5].classifier;
        GlobalConeData.confidence6 = DeCode[5].confidence;
    }
    if(Number > 6)
    {
        xyz2xy(mtxLeft, DeCode[6], xy, 0.3f);
        GlobalConeData.x7 = xy.x;
        GlobalConeData.y7 = xy.y;
        GlobalConeData.class7 = DeCode[6].classifier;
        GlobalConeData.confidence7 = DeCode[6].confidence;
    }
    if(Number > 7)
    {   
        xyz2xy(mtxLeft, DeCode[7], xy, 0.3f);        
        GlobalConeData.x8 = xy.x;
        GlobalConeData.y8 = xy.y;
        GlobalConeData.class8 = DeCode[7].classifier;
        GlobalConeData.confidence8 = DeCode[7].confidence;
    }
    
    // Set Flag to make sure data is sent out via openDlv message
    ConeDataCalculated = true;

}