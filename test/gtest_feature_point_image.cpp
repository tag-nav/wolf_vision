//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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


#include <string>
#include <core/utils/utils_gtest.h>
#include <core/sensor/sensor_base.h>

#include "vision/feature/feature_point_image.h"

using namespace wolf;
using namespace cv;

class FeaturePointImage_test : public testing::Test
{
    public:
        cv::KeyPoint cv_kp_;
        WKeyPoint kp_;
        Eigen::Vector2d pix_;
        std::string object_type_;
        Eigen::Matrix2d cov_;
        
        void SetUp() override
        {
            cv_kp_ = cv::KeyPoint(0.0, 1.0, 0);
            kp_ = WKeyPoint(cv_kp_);
            cov_ = Eigen::Matrix2d::Identity();
        }
};

TEST_F(FeaturePointImage_test, constructor)
{
    FeaturePointImagePtr f1 = std::make_shared<FeaturePointImage>(kp_, cov_);
    ASSERT_EQ(f1->getType(), "FeaturePointImage");
}

TEST_F(FeaturePointImage_test, getter_setters)
{
    FeaturePointImagePtr f1 = std::make_shared<FeaturePointImage>(kp_, cov_);
    ASSERT_EQ(f1->getMeasurement()(0), 0.0);

    cv::KeyPoint cv_kp_bis = cv::KeyPoint(2.0, 3.0, 0);
    WKeyPoint kp_bis(cv_kp_bis);
    f1->setKeyPoint(kp_bis);

    // setting the keypoint changes the feature measurement as it should
    ASSERT_EQ(f1->getMeasurement()(0), 2.0);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

