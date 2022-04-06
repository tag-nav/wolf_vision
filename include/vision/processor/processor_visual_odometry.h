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

#ifndef INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_
#define INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_


// Opencv includes
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

// wolf includes
#include "core/math/rotations.h"
#include "core/processor/processor_tracker.h"
#include "core/processor/track_matrix.h"
#include "vision/math/pinhole_tools.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/capture/capture_image.h"
#include "vision/feature/feature_point_image.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/factor/factor_pixel_hp.h"

// #include "vision/processor/klt_tracking.hpp"



namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorVisualOdometry);

struct KltParams
{
    int tracker_width_;
    int tracker_height_;
    double klt_max_err_;
    int nlevels_pyramids_;
    cv::TermCriteria crit_;
};

struct ParamsProcessorVisualOdometry : public ParamsProcessorTracker
{
    KltParams klt_params_;

    ParamsProcessorVisualOdometry() = default;
    ParamsProcessorVisualOdometry(std::string _unique_name, const ParamsServer& _server)
    {
        klt_params_.tracker_width_        = _server.getParam<int>(prefix + _unique_name + "/klt_params/tracker_width");
        klt_params_.tracker_height_       = _server.getParam<int>(prefix + _unique_name + "/klt_params/tracker_height");
        klt_params_.klt_max_err_          = _server.getParam<double>(prefix + _unique_name + "/klt_params/klt_max_err");
        klt_params_.nlevels_pyramids_ = _server.getParam<int>(prefix + _unique_name + "/klt_params/nlevels_pyramids");
        klt_params_.crit_                 = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);  // everybody uses this defaults...
    }
    std::string print() const override
    {

        return "Hello! TODO";
    }
};

WOLF_PTR_TYPEDEFS(ProcessorVisualOdometry);

class ProcessorVisualOdometry : public ProcessorTracker
{
    public:
        ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_visual_odometry);
        ~ProcessorVisualOdometry() override {};

        WOLF_PROCESSOR_CREATE(ProcessorVisualOdometry, ParamsProcessorVisualOdometry);

    protected:
        ParamsProcessorVisualOdometryPtr params_visual_odometry_;

        TrackMatrix track_matrix_;

        Matrix2d pixel_cov_;

        // A few casted smart pointers
        CaptureImagePtr capture_image_last_;
        CaptureImagePtr capture_image_incoming_;
        CaptureImagePtr capture_image_origin_;
        SensorCameraPtr sen_cam_;

    private:
        int frame_count_;

        // detector
        cv::Ptr<cv::FeatureDetector> detector_;

        // camera
        cv::Mat Kcv_;

        // bookeeping
        TracksMap tracks_map_li_matched_;
        CaptureBasePtr origin_prev_;


    public:

        void configure(SensorBasePtr _sensor) override;

        /** Pre-process incoming Capture, see ProcessorTracker
         */
        void preProcess() override;

        /** Post-process, see ProcessorTracker
         */
        virtual void postProcess() override;

        /** \brief Vote for KeyFrame generation, see ProcessorTracker
         */
        bool voteForKeyFrame() const override;

        /** \brief Tracker function
         * \return The number of successful tracks.
         *
         * see ProcessorTracker
         */
        unsigned int processKnown() override;


        /** \brief Process new Features or Landmarks
         *
         */
        unsigned int processNew(const int& _max_features) override;


        /**\brief Creates and adds factors from last_ to origin_
         *
         */
        void establishFactors() override;

        /**\brief Emplace a landmark corresponding to a track and initialize it with triangulation.
         *
         */
        LandmarkBasePtr emplaceLandmark(FeatureBasePtr _feature_ptr);


        /** \brief Advance the incoming Capture to become the last.
         *
         * see ProcessorTracker
         */
        void advanceDerived() override;


        /** \brief Reset the tracker using the \b last Capture as the new \b origin.
         */
        void resetDerived() override;

        /** \brief Implementation of pyramidal KLT with openCV
         */
        TracksMap kltTrack(cv::Mat img_prev, cv::Mat img_curr, KeyPointsMap &mwkps_prev, KeyPointsMap &mwkps_curr);

        /** \brief Implementation of 5 point algorithm with openCV, remove outliers from the tracks map
         */
        bool computeEssential(KeyPointsMap mwkps_prev, KeyPointsMap mwkps_curr, TracksMap &tracks_prev_curr, cv::Mat &E);

        void setParams(const ParamsProcessorVisualOdometryPtr _params);

        TrackMatrix getTrackMatrix(){return track_matrix_;}

};


} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_ */
