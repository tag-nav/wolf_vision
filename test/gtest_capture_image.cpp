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

        cv::KeyPoint cv_kp0_;
        cv::KeyPoint cv_kp1_;
        cv::KeyPoint cv_kp2_;

        WKeyPoint wkp0_;
        WKeyPoint wkp1_;
        WKeyPoint wkp2_;

        void SetUp() override
        {
            // to be sure that the counter start from zero each time a new test is created
            // not really usefull outside of this test
            WKeyPoint::resetIdCount();
            img_ = cv::Mat::eye(4, 4, CV_8UC1);

            cv_kp0_ = cv::KeyPoint(0.0, 0.0, 0);
            cv_kp1_ = cv::KeyPoint(1.0, 0.0, 0);
            cv_kp2_ = cv::KeyPoint(2.0, 0.0, 0);
            wkp0_ = WKeyPoint  (cv_kp0_);
            wkp1_ = WKeyPoint  (cv_kp1_);
            wkp2_ = WKeyPoint  (cv_kp2_);
        }
};

TEST_F(CaptureImage_test, WKeyPoint_class)
{
    // WKeyPoint ids start from 3 since because the default constructor 
    // is called in the declaration of CaptureImage_test attributes
    ASSERT_EQ(wkp0_.getId(), 1);
    ASSERT_EQ(wkp1_.getId(), 2);
    ASSERT_EQ(wkp2_.getId(), 3);

    ASSERT_EQ(wkp0_.getCvKeyPoint().pt.x, 0.0);
    ASSERT_EQ(wkp1_.getCvKeyPoint().pt.x, 1.0);
    ASSERT_EQ(wkp2_.getCvKeyPoint().pt.x, 2.0);
    ASSERT_EQ(wkp0_.getEigenKeyPoint()(0), 0.0);
    ASSERT_EQ(wkp1_.getEigenKeyPoint()(0), 1.0);
    ASSERT_EQ(wkp2_.getEigenKeyPoint()(0), 2.0);

    wkp0_.setCvKeyPoint(cv_kp1_);
    ASSERT_EQ(wkp0_.getCvKeyPoint().pt.x, 1.0);
    ASSERT_EQ(wkp0_.getEigenKeyPoint()(0), 1.0);

    cv::Mat desc = 3*cv::Mat::eye(4, 4, CV_8UC1);
    wkp0_.setDescriptor(desc);
    ASSERT_EQ(wkp0_.getDescriptor().at<uchar>(0,0), 3);
}


TEST_F(CaptureImage_test, capture_image_type)
{
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);

    ASSERT_EQ(c->getType(), "CaptureImage");
}

TEST_F(CaptureImage_test, getter_setters_img)
{
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);
    cv::Mat temp = c->getImage();
    ASSERT_EQ(temp.at<uchar>(0,0), 1);

    // expected behavior: changing the external image does not change the image
    // inside the capture as they share the same underlying data array (as "=" operators are used)
    temp = 3*cv::Mat::eye(4, 4, CV_8UC1);
    ASSERT_EQ(temp.at<uchar>(0,0), 3);

    cv::Mat temp2 = 6*cv::Mat::eye(4, 4, CV_8UC1);
    c->setImage(temp2);
    ASSERT_EQ(c->getImage().at<uchar>(0,0), 6);
}


TEST_F(CaptureImage_test, add_remove_key_points)
{
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);
    c->addKeyPoint(wkp0_);
    c->addKeyPoint(wkp1_);
    c->addKeyPoint(wkp2_);

    ASSERT_EQ(c->getKeyPoints().size(), 3);

    c->removeKeyPoint(wkp0_.getId());
    ASSERT_EQ(c->getKeyPoints().size(), 2);

    c->removeKeyPoint(wkp1_);
    ASSERT_EQ(c->getKeyPoints().size(), 1);

    // only wkp2 is left

    ASSERT_EQ(c->getKeyPoints().at(3).getId(), 3);
    ASSERT_EQ(c->getKeyPoints().at(3).getCvKeyPoint().pt.x, 2.0);

    // create a new WKeyPoint and add it to the keypoint map
    // the new WKeyPoint ID is therefore 3 as well as its key in the map
    c->addKeyPoint(cv_kp0_);  
    ASSERT_EQ(c->getKeyPoints().at(4).getId(), 4);
    ASSERT_EQ(c->getKeyPoints().at(4).getCvKeyPoint().pt.x, 0.0);


}

TEST_F(CaptureImage_test, add_remove_key_points_using_map)
{
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);
    // Now using a KeyPointsMap
    KeyPointsMap mapwkps;
    mapwkps.insert(std::pair<size_t, WKeyPoint>(wkp0_.getId(), wkp0_));
    mapwkps.insert(std::pair<size_t, WKeyPoint>(wkp1_.getId(), wkp1_));
    mapwkps.insert(std::pair<size_t, WKeyPoint>(wkp2_.getId(), wkp2_));

    c->addKeyPoints(mapwkps);
    ASSERT_EQ(c->getKeyPoints().size(), 3);
    ASSERT_EQ(c->getKeyPoints().at(3).getId(), 3);
}




TEST_F(CaptureImage_test, add_remove_key_point_vectors)
{
    // Same as add_remove_key_points but with the vector argument
    CaptureImagePtr c = std::make_shared<CaptureImage>(0, nullptr, img_);
    std::vector<WKeyPoint> wkp_vec = {wkp0_, wkp1_, wkp2_};
    c->addKeyPoints(wkp_vec);
    ASSERT_EQ(c->getKeyPoints().size(), 3);

    // adding the same wolf WKeyPoint vector is indepotent 
    c->addKeyPoints(wkp_vec);
    ASSERT_EQ(c->getKeyPoints().size(), 3);

    // but adding new cv::KeyPoint will create new WKeyPoint
    std::vector<WKeyPoint> cv_kp_vec = {cv_kp0_, cv_kp1_, cv_kp2_};
    c->addKeyPoints(cv_kp_vec);
    ASSERT_EQ(c->getKeyPoints().size(), 6);
    // at position 4 is the new WKeyPoint created from cv_kp1_
    ASSERT_EQ(c->getKeyPoints().at(5).getId(), 5);
    ASSERT_EQ(c->getKeyPoints().at(5).getCvKeyPoint().pt.x, 1.0);
}





int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

