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


// wolf plugin includes
#include "vision/internal/config.h"
#include "vision/math/pinhole_tools.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/capture/capture_image.h"
#include "vision/feature/feature_point_image.h"
#include "vision/landmark/landmark_hp.h"
#include "vision/factor/factor_pixel_hp.h"
#include "vision/processor/active_search.h"

// wolf includes
#include <core/math/rotations.h>
#include <core/math/SE3.h>
#include <core/state_block/state_composite.h>
#include <core/processor/processor_tracker.h>
#include <core/processor/track_matrix.h>
#include <core/processor/motion_provider.h>

// Opencv includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/eigen.hpp>



namespace wolf{

/** \brief Buffer of VectorComposite
 *
 * Object and functions to manage a buffer of VectorComposite objects.
 * Used to hold a memory of origin->last relative poses.
 */
class BufferVectorCompositePtr : public Buffer<std::shared_ptr<VectorComposite>> { };


WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorVisualOdometry);

struct ParamsProcessorVisualOdometry : public ParamsProcessorTracker, public ParamsMotionProvider
{
    struct RansacParams
    {
        double prob;
        double thresh;
    };

    struct KltParams
    {
        int patch_width;
        int patch_height;
        double max_err;
        int nlevels_pyramids;
        cv::TermCriteria criteria;
    };

    struct FastParams
    {
        int threshold;
        bool non_max_suppresion;
    };

    struct GridParams
    {
        unsigned int nbr_cells_h;
        unsigned int nbr_cells_v;
        unsigned int margin;
        unsigned int separation;
    };

    struct EqualizationParams
    {
            unsigned int method; // 0: none; 1: average; 2: histogram; 3: CLAHE
            // note: cv::histogramEqualization() has no tuning params
            struct AverageParams
            {
                    int median;
            } average;
            struct ClaheParams
            {
                    double clip_limit;
                    cv::Size2i tile_grid_size;
            } clahe;
    };

    struct LandmarkCreationParams
    {
        unsigned int min_track_length_for_landmark;
        double min_pixel_dist;
    };

    RansacParams ransac;
    KltParams klt;
    FastParams fast;
    GridParams grid;
    EqualizationParams equalization;
    LandmarkCreationParams lmk_creation;
    double std_pix;

    ParamsProcessorVisualOdometry() = default;
    ParamsProcessorVisualOdometry(std::string _unique_name, const ParamsServer& _server):
        ParamsProcessorTracker(_unique_name, _server),
        ParamsMotionProvider(_unique_name, _server)
    {
        std_pix = _server.getParam<double>(prefix + _unique_name + "/std_pix");

        equalization.method = _server.getParam<unsigned int>(prefix + _unique_name + "/equalization/method");
        switch (equalization.method)
        {
            case 0: break;
            case 1:
                equalization.average.median = _server.getParam<unsigned int>(prefix + _unique_name + "/equalization/average/median");
                break;
            case 2:
                // note: cv::histogramEqualization() has no tuning params
                break;
            case 3:
                equalization.clahe.clip_limit = _server.getParam<double>(prefix + _unique_name + "/equalization/clahe/clip_limit");
                vector<int> grid_size = _server.getParam<vector<int>>(prefix + _unique_name + "/equalization/clahe/tile_grid_size");
                equalization.clahe.tile_grid_size.width  = grid_size[0];
                equalization.clahe.tile_grid_size.height = grid_size[1];
                break;
        }

        ransac.prob   = _server.getParam<double>(prefix + _unique_name + "/ransac/prob");
        ransac.thresh = _server.getParam<double>(prefix + _unique_name + "/ransac/thresh");

        klt.patch_width        = _server.getParam<int>    (prefix + _unique_name + "/klt/patch_width");
        klt.patch_height       = _server.getParam<int>    (prefix + _unique_name + "/klt/patch_height");
        klt.max_err            = _server.getParam<double> (prefix + _unique_name + "/klt/max_err");
        klt.nlevels_pyramids   = _server.getParam<int>    (prefix + _unique_name + "/klt/nlevels_pyramids");
        klt.criteria           = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);  // everybody uses this defaults...

        fast.threshold         = _server.getParam<int>    ( prefix + _unique_name + "/fast/threshold");
        fast.non_max_suppresion = _server.getParam<bool>  (prefix + _unique_name + "/fast/non_max_suppresion");

        grid.nbr_cells_h   = _server.getParam<unsigned int>(prefix + _unique_name + "/grid/nbr_cells_h");
        grid.nbr_cells_v   = _server.getParam<unsigned int>(prefix + _unique_name + "/grid/nbr_cells_v");
        grid.margin        = _server.getParam<unsigned int>(prefix + _unique_name + "/grid/margin");
        grid.separation    = _server.getParam<unsigned int>(prefix + _unique_name + "/grid/separation");

        lmk_creation.min_track_length_for_landmark = _server.getParam<unsigned int>(prefix + _unique_name + "/lmk_creation/min_track_length_for_landmark");
        lmk_creation.min_pixel_dist = _server.getParam<double>(prefix + _unique_name + "/lmk_creation/min_pixel_dist");
        

    }
    std::string print() const override
    {
        return ParamsProcessorTracker::print()                                                      + "\n"
            + "equalization.method:       " + std::to_string(equalization.method)                   + "\n"
            + "ransac.prob:               " + std::to_string(ransac.prob)                           + "\n"
            + "ransac.thresh:             " + std::to_string(ransac.thresh)                         + "\n"
            + "klt.patch_width:           " + std::to_string(klt.patch_width)                       + "\n"
            + "klt.patch_height:          " + std::to_string(klt.patch_height)                      + "\n"
            + "klt.max_err:               " + std::to_string(klt.max_err)                           + "\n"
            + "klt.nlevels_pyramids:      " + std::to_string(klt.nlevels_pyramids)                  + "\n"
            + "fast.threshold:            " + std::to_string(fast.threshold)                        + "\n"
            + "fast.non_max_suppresion:   " + std::to_string(fast.non_max_suppresion)               + "\n"
            + "grid.nbr_cells_h:          " + std::to_string(grid.nbr_cells_h)                      + "\n"
            + "grid.nbr_cells_v:          " + std::to_string(grid.nbr_cells_v)                      + "\n"
            + "grid.margin:               " + std::to_string(grid.margin)                           + "\n"
            + "grid.separation:           " + std::to_string(grid.separation)                       + "\n"
            + "lmk_creation.min_track_length_for_landmark:   " + std::to_string(lmk_creation.min_track_length_for_landmark)   + "\n"
            + "lmk_creation.min_pixel_dist:   " + std::to_string(lmk_creation.min_pixel_dist)   + "\n";
    }
};

WOLF_PTR_TYPEDEFS(ProcessorVisualOdometry);

class ProcessorVisualOdometry : public ProcessorTracker, public MotionProvider
{
    public:
        ProcessorVisualOdometry(ParamsProcessorVisualOdometryPtr _params_visual_odometry);
        virtual ~ProcessorVisualOdometry() override {};

        // MotionProvider class pure virtual methods
        VectorComposite getState(const StateStructure& _structure = "") const override;
        TimeStamp getTimeStamp( ) const override;
        VectorComposite getState(const TimeStamp& _ts, const StateStructure& _structure = "") const override;
        VectorComposite getRelativePoseOriginLast() const;

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
        // camera
        cv::Mat Kcv_;

        // bookeeping
        TracksMap tracks_map_li_matched_;

        // buffer of origin->last camera relative poses mapping origin to last
        BufferVectorCompositePtr buffer_pose_cam_ol_;


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
         * 
         * Implementation: Use rays of features detections in last frame and create a landmark at 1 meter (arbitrary) 
         */
        LandmarkBasePtr emplaceLandmarkNaive(FeaturePointImagePtr _feature_ptr);

        /**
         * \brief Emplace a landmark corresponding to a track and initialize it with triangulation.
         * \param _feature_ptr a pointer to the feature used to create the new landmark
         * \param _track_kf track with only features associated to keyframes that maye be used for initialisation
         * \return a pointer to the created landmark. If null, the triangulation failed due to low parallax
         */
        LandmarkBasePtr emplaceLandmarkTriangulation(FeaturePointImagePtr _feature_ptr, Track _track_kf);

                /**
         * \brief Emplace a landmark corresponding to a track and initialize it with triangulation.
         * \param _feature_ptr a pointer to the feature used to create the new landmark
         * \param _track_kf track with only features associated to keyframes that maye be used for initialisation
         * \return the triangulated point in P3 homogeneous coordinates
         * 
         * Implementation: try to triangulate a new landmark based on previous frames estimates.
         * Apply a numerical test to asses if parallax is enough.
         */
        bool tryTriangulationFromFeatures(FeaturePointImagePtr _feat, Track _track_kf, Eigen::Vector4d&);


        Eigen::Isometry3d getTcw(TimeStamp _ts) const;

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
        bool filterWithEssential(const KeyPointsMap mwkps_prev, 
                                 const KeyPointsMap mwkps_curr, 
                                 TracksMap &tracks_prev_curr, 
                                 cv::Mat &E, 
                                 VectorComposite &_pose_prev_curr);

        /** \brief Merge track maps between moments prev->curr and curr->next to give a track map between prev->next.
         * 
         * \param tracks_prev_curr prev->curr track map 
         * \param tracks_curr_next curr->next track map 
         * \return merged track prev->next
         */
        static TracksMap mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next);

        const TrackMatrix& getTrackMatrix() const {return track_matrix_;}

        ///////////////////////////////////////
        // MotionProvider related methods

        VectorComposite getStateFromRelativeOriginLast(VectorComposite co_pose_cl) const;

        static VectorComposite pose_from_essential_matrix(const cv::Mat& _E, 
                                                          const std::vector<cv::Point2d>& p2d_prev, 
                                                          const std::vector<cv::Point2d>& p2d_curr, 
                                                          const cv::Mat& _K,
                                                          cv::Mat& cvMask);
        ///////////////////////////////////////


        /** \brief sequence of heuristics to decide if a track is worthy of becoming a landmark
         * 
         */
        bool new_landmark_is_viable(int track_id);

    private:
        void retainBest(std::vector<cv::KeyPoint> &_keypoints, int n);

        /* Equalize image for better detection and tracking
        * available methods:
        *      0. none
        *      1. average
        *      2. opencv: histogram_equalization
        *      3. opencv: CLAHE
        */
        void equalize_img(cv::Mat &img_incoming);

        void detect_keypoints_empty_grid(cv::Mat img_incoming, CaptureImagePtr capture_image_incoming);

        void filter_last_incoming_tracks_from_ransac_result(const TracksMap& tracks_last_incoming, const KeyPointsMap& mwkps_incoming, const TracksMap& tracks_origin_incoming,
                                                            TracksMap& tracks_last_incoming_filtered, KeyPointsMap& mwkps_incoming_filtered);
        
        void detect_keypoints_in_empty_grid_cells(cv::Mat img_last, const TracksMap& tracks_last_incoming_filtered, const KeyPointsMap& mwkps_last, 
                                                  std::vector<cv::KeyPoint>& kps_last_new, KeyPointsMap& mwkps_last_filtered);

};

} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_VISUAL_ODOMETRY_H_ */
