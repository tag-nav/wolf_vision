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

#ifndef FEATURE_POINT_IMAGE_H
#define FEATURE_POINT_IMAGE_H

// OpenCV includes
#include <opencv2/core.hpp>

//Wolf includes
#include "core/feature/feature_base.h"
#include "vision/capture/capture_image.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(FeaturePointImage);
    
//class FeaturePointImage
class FeaturePointImage : public FeatureBase
{
    private:
        WKeyPoint kp_;
        
        // id of the landmark to which the feature is associated (maybe in processor no?) 
        unsigned int lmk_associated_id_;

    public:
        /// Constructor from OpenCV measured keypoint
        FeaturePointImage(const WKeyPoint& _keypoint,
                          const Eigen::Matrix2d& _meas_covariance);

       ~FeaturePointImage() override;

        const WKeyPoint& getKeyPoint() const {return kp_;}
        void setKeyPoint(const WKeyPoint& _kp);

        unsigned int getLandmarkAssociatedId() const {return lmk_associated_id_;}
        void setLandmarkAssociatedId(unsigned int _lmk_id) {lmk_associated_id_ = _lmk_id;}
};

inline void FeaturePointImage::setKeyPoint(const WKeyPoint& _kp)
{
    kp_ = _kp;
    measurement_(0) = _kp.getCvKeyPoint().pt.x;
    measurement_(1) = _kp.getCvKeyPoint().pt.y;
}

} // namespace wolf

#endif // FEATURE_POINT_IMAGE_H
