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
int xyz2xy(cv::Mat Q, float x, float y, float z, cv::Point2f& xy, float radius);

// Function to process 3d points in a frame into 2d coordinates
int xyz2xy(cv::Mat Q, float x, float y, float z, cv::Point2f &xy, float radius){

  float Cx = float(-Q.at<double>(0,3));
  float Cy = float(-Q.at<double>(1,3));
  float f = float(Q.at<double>(2,3));
  float a = float(Q.at<double>(3,2));
  float b = float(Q.at<double>(3,3));
  float d = (f - z * b ) / ( z * a);  
  
  // Why does one calculate everything as floats and stores it as an int?
  xy.x = float(x * ( d * a + b ) + Cx);
  xy.y = float(y * ( d * a + b ) + Cy);
  
  // Probably not necessary in our case
  return int(radius * ( d * a + b ));
}

// Take the cone data and prepare to send out, needs to be called after the network is done with the calculation of a frame
cv::Point2f CalculateCone2xy(bbox_t cones){

    cv::Point2f xy = {0.0, 0.0};
    xyz2xy(mtxLeft, cones.x_3d, cones.y_3d, cones.z_3d, xy, 0.3f);
}