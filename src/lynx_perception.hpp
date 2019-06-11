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
 */

#ifndef LYNX_PERCEPTION
#define LYNX_PERCEPTION

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "cfsd-extended-message-set.hpp"
#include "../darknet/include/yolo_v2_class.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

extern opendlv::cfsdPerception::Cones8 GlobalConeData;
extern cv::Mat GlobalFrame;
extern bool ConeDataCalculated;
extern bool FrameAvailable;

typedef struct {
    uint8_t classifier;
    float x_center;
    float y_center;
    float z_center;
    double width;
    double height;
    float confidence;
    int num;
}detection;

// Function to Calculate 3d Conces Found in Frame into 2d birView coordinates
cv::Point2f CalculateCone2xy(bbox_t cones);
#endif 