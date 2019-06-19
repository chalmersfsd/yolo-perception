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

//Define intrinsic camera parameters inside matrix (left/right)
cv::Mat mtxLeft = (cv::Mat_<double>(3, 3) <<
    349.891, 0, 334.352,
    0, 349.891, 187.937,
    0, 0, 1);

cv::Mat mtxRight = (cv::Mat_<double>(3, 3) <<
    350.112, 0, 345.88,
    0, 350.112, 189.891,
    0, 0, 1);

// Declaration of local function
cv::Point2f xyz2xy(cv::Mat Q, float x, float y, float z);

// Function to process 3d points in a frame into 2d coordinates
cv::Point2f xyz2xy(cv::Mat Q, float x, float y, float z){

  cv::Point2f xy = {0.0, 0.0};
  float Cx = float(-Q.at<double>(0,2));
  float Cy = float(-Q.at<double>(1,1));
  float f = float(Q.at<double>(2,3));

  xy.x = (z*x-z*Cx)/f;
  xy.y = (z*y-z*Cy)/f;

  return xy;
}

// Wrapper function to insert calibration matrix
cv::Point2f CalculateCone2xy(bbox_t cones){

    cv::Point2f xy = xyz2xy(mtxLeft, cones.x, cones.y, cones.z_3d);

    return xy;
}
