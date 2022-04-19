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
#include "vision/processor/active_search.h"



namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorVisualOdometry);

struct ParamsProcessorVisualOdometry : public ParamsProcessorTracker
{
    struct RansacParams
    {
        double ransac_prob_;
        double ransac_thresh_;
    };

    struct KltParams
    {
        int patch_width_;
        int patch_height_;
        double klt_max_err_;
        int nlevels_pyramids_;
        cv::TermCriteria criteria_;
    };

    struct FastParams
    {
        int threshold_fast_;
        bool non_max_suppresion_;
    };

    struct GridParams
    {
        unsigned int nbr_cells_h_;
        unsigned int nbr_cells_v_;
        unsigned int margin_;
        unsigned int separation_;
    };

    double std_pix_;
    RansacParams ransac_params_;
    KltParams klt_params_;
    FastParams fast_params_;
    GridParams grid_params_;
    unsigned int max_nb_tracks_;
    unsigned int min_track_length_for_landmark_;

    ParamsProcessorVisualOdometry() = default;
    ParamsProcessorVisualOdometry(std::string _unique_name, const ParamsServer& _server):
        ParamsProcessorTracker(_unique_name, _server)
    {
        std_pix_ = _server.getParam<int>(prefix + _unique_name + "/std_pix");

        ransac_params_.ransac_prob_   = _server.getParam<double>(prefix + _unique_name + "/ransac_params/ransac_prob");
        ransac_params_.ransac_thresh_ = _server.getParam<double>(prefix + _unique_name + "/ransac_params/ransac_thresh");

        klt_params_.patch_width_        = _server.getParam<int>(prefix + _unique_name + "/klt_params/patch_width");
        klt_params_.patch_height_       = _server.getParam<int>(prefix + _unique_name + "/klt_params/patch_height");
        klt_params_.klt_max_err_        = _server.getParam<double>(prefix + _unique_name + "/klt_params/klt_max_err");
        klt_params_.nlevels_pyramids_   = _server.getParam<int>(prefix + _unique_name + "/klt_params/nlevels_pyramids");
        klt_params_.criteria_           = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);  // everybody uses this defaults...

        fast_params_.threshold_fast_     = _server.getParam<int>( prefix + _unique_name + "/fast_params/threshold_fast");
        fast_params_.non_max_suppresion_ = _server.getParam<bool>(prefix + _unique_name + "/fast_params/non_max_suppresion");

        grid_params_.nbr_cells_h_   = _server.getParam<unsigned int>(prefix + _unique_name + "/grid_params/nbr_cells_h");
        grid_params_.nbr_cells_v_   = _server.getParam<unsigned int>(prefix + _unique_name + "/grid_params/nbr_cells_v");
        grid_params_.margin_        = _server.getParam<unsigned int>(prefix + _unique_name + "/grid_params/margin");
        grid_params_.separation_    = _server.getParam<unsigned int>(prefix + _unique_name + "/grid_params/separation");

        max_nb_tracks_ = _server.getParam<unsigned int>(prefix + _unique_name + "/max_nb_tracks");
        min_track_length_for_landmark_ = _server.getParam<unsigned int>(prefix + _unique_name + "/min_track_length_for_landmark");

    }
    std::string print() const override
    {
        return ParamsProcessorTracker::print()                                                                   + "\n"
            + "ransac_params_.ransac_prob_:      " + std::to_string(ransac_params_.ransac_prob_)                 + "\n"
            + "ransac_params_.ransac_thresh_:    " + std::to_string(ransac_params_.ransac_thresh_)               + "\n"
            + "klt_params_.tracker_width_:       " + std::to_string(klt_params_.patch_width_)                    + "\n"
            + "klt_params_.tracker_height_:      " + std::to_string(klt_params_.patch_height_)                   + "\n"
            + "klt_params_.klt_max_err_:         " + std::to_string(klt_params_.klt_max_err_)                    + "\n"
            + "klt_params_.nlevels_pyramids_:    " + std::to_string(klt_params_.nlevels_pyramids_)               + "\n"
            + "fast_params_.threshold_fast_:     " + std::to_string(fast_params_.threshold_fast_)                + "\n"
            + "fast_params_.non_max_suppresion_: " + std::to_string(fast_params_.non_max_suppresion_)            + "\n"
            + "grid_params_.nbr_cells_h_:        " + std::to_string(grid_params_.nbr_cells_h_)                   + "\n"
            + "grid_params_.nbr_cells_v_:        " + std::to_string(grid_params_.nbr_cells_v_)                   + "\n"
            + "grid_params_.margin_:             " + std::to_string(grid_params_.margin_)                        + "\n"
            + "grid_params_.separation_:         " + std::to_string(grid_params_.separation_)                    + "\n"
            + "max_nb_tracks_:                   " + std::to_string(max_nb_tracks_)                              + "\n"
            + "min_track_length_for_landmark_:   " + std::to_string(min_track_length_for_landmark_)              + "\n";
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

        // detector
        cv::Ptr<cv::FeatureDetector> detector_;

        // A few casted smart pointers
        CaptureImagePtr capture_image_last_;
        CaptureImagePtr capture_image_incoming_;
        CaptureImagePtr capture_image_origin_;
        SensorCameraPtr sen_cam_;

        ActiveSearchGrid cell_grid_;

    private:
        int frame_count_;

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

        /**
         * \brief Implementation of pyramidal KLT with openCV
         * \param img_prev previous image
         * \param img_curr current image
         * \param mwkps_prev keypoints in previous image
         * \param mwkps_curr keypoints in current image, those tracked from the previous image
         * \return a map with track associations
         */
        TracksMap kltTrack(const cv::Mat img_prev, const cv::Mat img_curr, const KeyPointsMap &mwkps_prev, KeyPointsMap &mwkps_curr);

        /** \brief Remove outliers from the tracks map with a RANSAC 5-points algorithm implemented on openCV
         */
        bool filterWithEssential(const KeyPointsMap mwkps_prev, const KeyPointsMap mwkps_curr, TracksMap &tracks_prev_curr, cv::Mat &E);

        /** \brief Tool to merge tracks 
         */
        static TracksMap mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next);

        void setParams(const ParamsProcessorVisualOdometryPtr _params);

        const TrackMatrix& getTrackMatrix() const {return track_matrix_;}

    private:
        void retainBest(std::vector<cv::KeyPoint> &_keypoints, int n);

};

} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_ */
