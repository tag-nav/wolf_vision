//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
/**
 * \file gtest_capture_image.cpp
 *
 *  Created on: March 31, 2022
 *      \author: mfourmy
 */

#include <string>
#include <core/utils/utils_gtest.h>
#include <core/sensor/sensor_base.h>

#include "vision/capture/capture_image.h"

using namespace wolf;
using namespace cv;

class CaptureImage_test : public testing::Test
{
    public:
        cv::Mat img_;
        std::string object_type_;
        
        void SetUp() override
        {
            img_ = cv::Mat::eye(4, 4, CV_64F);
        }
};

TEST_F(CaptureImage_test, type)
{
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);

    ASSERT_EQ(c->getType(), "CaptureImage");
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

