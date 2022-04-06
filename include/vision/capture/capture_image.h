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
 * \file gtest_factor_epipolar.cpp
 *
 *  Created on: March 31, 2022
 *      \author: mfourmy
 */

#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

// OpenCV includes
#include <opencv2/core.hpp>


//Wolf includes
#include <core/capture/capture_base.h>
#include "vision/sensor/sensor_camera.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(WKeyPoint);
class WKeyPoint 
{
    private:
        static size_t id_count_;  // class attribute automatically incremented when a new WKeyPoint object is instanciated 
        size_t id_;
        cv::KeyPoint cv_kp_;
        cv::Mat desc_;
    
    public:

        WKeyPoint();
        WKeyPoint(const cv::KeyPoint& _cv_kp);

        size_t getId() const {return id_;}
        void setId(size_t _id) {id_ = _id;}

        cv::KeyPoint getCvKeyPoint() const {return cv_kp_;}
        void setCvKeyPoint(cv::KeyPoint _cv_kp) {cv_kp_ = _cv_kp;}

        cv::Mat getDescriptor() const {return desc_;}
        void setDescriptor(cv::Mat _desc) {desc_ = _desc;}

        // Only used for gtest, should be moved
        static void resetIdCount() {id_count_ = 0;}
};



typedef std::unordered_map<size_t, WKeyPoint> KeyPointsMap;
typedef std::unordered_map<size_t, size_t> TracksMap;

// Set ClassPtr, ClassConstPtr and ClassWPtr typedefs;
WOLF_PTR_TYPEDEFS(CaptureImage);
    
/**
 * \brief class CaptureImage
 *
 * This class stores a cv::Mat image, with keypoints and descriptors defined in the OpenCV format.
 * This encapsulation allows this Capture to be used in OpenCV with ease.
 */
class CaptureImage : public CaptureBase
{
    private:
        cv::Mat img_;

        // Keypoints associated to the capture 
        KeyPointsMap mwkps_;

        // descriptors of the keypoints if necessary. 
        // number of rows ==  keypoints_.size() if intialized
        cv::Mat descriptors_;

        // keeps track from the origin capture (origin->incoming): used for outlier detection
        TracksMap tracks_origin_;

        // keeps track from the previous capture (last->incoming): by the rest of the processor to populate the tack matrix
        TracksMap tracks_prev_;

        bool last_was_repopulated_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _data_cv);
        ~CaptureImage() override;

        const cv::Mat& getImage() const;
        void setImage(const cv::Mat& _img);

        const KeyPointsMap& getKeyPoints() const {return mwkps_;}
        void setKeyPoints(const KeyPointsMap& _mwkps){mwkps_ = _mwkps;}

        const TracksMap& getTracksPrev() const {return tracks_prev_;}
        void setTracksPrev(const TracksMap& _tracks){tracks_prev_ = _tracks;}

        const TracksMap& getTracksOrigin() const {return tracks_origin_;}
        void setTracksOrigin(const TracksMap& _tracks){tracks_origin_ = _tracks;}

        bool getLastWasRepopulated(){return last_was_repopulated_;}
        void setLastWasRepopulated(bool _repopulated){last_was_repopulated_ = _repopulated;}

        void addKeyPoint(const WKeyPoint& _wkp);
        void addKeyPoint(const cv::KeyPoint& _cv_kp);

        void addKeyPoints(const std::vector<WKeyPoint>& _vec_wkp);
        void addKeyPoints(const std::vector<cv::KeyPoint>& _vec_cv_kp);
        void addKeyPoints(const KeyPointsMap& _mwkps);

        void removeKeyPoint(size_t _kp_id);
        void removeKeyPoint(const WKeyPoint& _wkp);

};

} // namespace wolf

#endif // CAPTURE_IMAGE_H
