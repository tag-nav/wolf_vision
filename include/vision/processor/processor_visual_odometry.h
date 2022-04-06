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

struct ParamsProcessorVisualOdometry : public ParamsProcessorTracker
{
	ParamsProcessorVisualOdometry() = default;
    ParamsProcessorVisualOdometry(std::string _unique_name, const ParamsServer& _server)
    {
        //
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
        SensorCameraPtr sen_cam_;

    private:
        int frame_count_;

        // detector
        int npoints_;
        double scale_factor_;
        int nlevels_pyramids_;
        cv::Ptr<cv::FeatureDetector> detector_;

        // tracker
        int tracker_width_;
        int tracker_height_;
        double klt_max_err_;
        int nlevels_pyramids_klt_;
        cv::TermCriteria crit_;

        int search_width_;
        int search_height_;
        int pyramid_level_;


        // camera
        cv::Mat Kcv_;

        // bookeeping
        TracksMap tracks_map_li_matched_;


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

        void setParams(const ParamsProcessorVisualOdometryPtr _params);

};

TracksMap klt_track(cv::Mat img_prev, cv::Mat img_curr, KeyPointsMap &mkps_prev, KeyPointsMap &mkps_curr,
           int search_width = 21, int search_height = 21, int pyramid_level = 3, float klt_max_err = 50.);


} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_ */
