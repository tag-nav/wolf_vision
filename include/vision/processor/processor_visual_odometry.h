//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023,2024 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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


// wolf plugin includes
#include "vision/math/pinhole_tools.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/capture/capture_image.h"
#include "vision/feature/feature_point_image.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/factor/factor_pixel_hp.h"
#include "vision/processor/active_search.h"

// wolf includes
#include <core/math/rotations.h>
#include <core/processor/processor_tracker.h>
#include <core/processor/track_matrix.h>

// Opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>



namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorVisualOdometry);

struct ParamsProcessorVisualOdometry : public ParamsProcessorTracker
{
    double std_pix;
    unsigned int min_track_length_for_landmark;

    ParamsProcessorVisualOdometry() = default;
    ParamsProcessorVisualOdometry(std::string _unique_name, const ParamsServer& _server):
        ParamsProcessorTracker(_unique_name, _server)
    {
        std_pix = _server.getParam<double>(prefix + _unique_name + "/std_pix");
        min_track_length_for_landmark = _server.getParam<unsigned int>(prefix + _unique_name + "/min_track_length_for_landmark");

        // [TODO] Add others (if needed)
    }
    std::string print() const override
    {
        return ParamsProcessorTracker::print()                                                      + "\n"
            // + "equalization.method:       " + std::to_string(equalization.method)                   + "\n"
            // + "ransac.prob:               " + std::to_string(ransac.prob)                           + "\n"
            // + "ransac.thresh:             " + std::to_string(ransac.thresh)                         + "\n"
            // + "klt.patch_width:           " + std::to_string(klt.patch_width)                       + "\n"
            // + "klt.patch_height:          " + std::to_string(klt.patch_height)                      + "\n"
            // + "klt.max_err:               " + std::to_string(klt.max_err)                           + "\n"
            // + "klt.nlevels_pyramids:      " + std::to_string(klt.nlevels_pyramids)                  + "\n"
            // + "fast.threshold:            " + std::to_string(fast.threshold)                        + "\n"
            // + "fast.non_max_suppresion:   " + std::to_string(fast.non_max_suppresion)               + "\n"
            // + "grid.nbr_cells_h:          " + std::to_string(grid.nbr_cells_h)                      + "\n"
            // + "grid.nbr_cells_v:          " + std::to_string(grid.nbr_cells_v)                      + "\n"
            // + "grid.margin:               " + std::to_string(grid.margin)                           + "\n"
            // + "grid.separation:           " + std::to_string(grid.separation)                       + "\n"
            + "min_track_length_for_landmark:   " + std::to_string(min_track_length_for_landmark)   + "\n";
    }
};

WOLF_PTR_TYPEDEFS(ProcessorVisualOdometry);

class ProcessorVisualOdometry : public ProcessorTracker
{
    public:
        ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_visual_odometry);
        virtual ~ProcessorVisualOdometry() override {};

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
        // camera
        cv::Mat Kcv_;

        // bookeeping
        TracksMap tracks_map_li_matched_;


    public:

        /**
         * \brief Get params from sensor to finish processor setup
         */
        void configure(SensorBasePtr _sensor) override;

        /** \brief Pre-process incoming Capture, see ProcessorTracker
         */
        void preProcess() override;

        /** \brief Post-process, see ProcessorTracker
         */
        virtual void postProcess() override;

        /** \brief Vote for KeyFrame generation, see ProcessorTracker
         */
        bool voteForKeyFrame() const override;

        /**
         * \brief Tracker function
         * \return The number of successful tracks.
         *
         * see ProcessorTracker
         */
        unsigned int processKnown() override;


        /**
         * \brief Process new Features
         * \param _max_features the maximum number of new feature tracks
         * \return the number of new tracks
         */
        unsigned int processNew(const int& _max_features) override;


        /**
         * \brief Creates and adds factors from last_ to origin_
         */
        void establishFactors() override;

        /**
         * \brief Emplace a landmark corresponding to a track and initialize it with triangulation.
         * \param _feature_ptr a pointer to the feature used to create the new landmark
         * \return a pointer to the created landmark
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

        /** \brief Tool to merge tracks 
         */
        static TracksMap mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next);

        CaptureImagePtr get_capture_image_last() {
            return capture_image_last_;
        }
        CaptureImagePtr get_capture_image_incoming() {
            return capture_image_incoming_;
        }
        CaptureImagePtr get_capture_image_origin() {
            return capture_image_origin_;
        }

};

} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_ */
