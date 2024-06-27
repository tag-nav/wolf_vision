#ifndef PROCESSOR_VO_UTILS_H_
#define PROCESSOR_VO_UTILS_H_

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
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>


namespace wolf{
namespace vo_utils {

// functions related to feature extraction
void retainBest(std::vector<cv::KeyPoint> &_keypoints, int n);

// functions related to feature tracking
TracksMap mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next);

TracksMap kltTrack(const ParamsProcessorVisualOdometryPtr _params_vo,
                   const cv::Mat& _img_prev, const cv::Mat& _img_curr,
                   const KeyPointsMap &_mwkps_prev, KeyPointsMap& _mwkps_curr);

// functions for evaluations
double getParallax(const Eigen::Vector4d& _pinhole_model, 
                   const KeyPointsMap& _mwkps_prev, const KeyPointsMap& _mwkps_curr, 
                   const TracksMap& _tracks_prev_curr);

double evalReprojError(const std::vector<Eigen::Vector3d>& pts3d, 
                       const std::vector<Eigen::Vector2d>& pts2d,
                       const Eigen::Vector4d& K_vec,
                       const Eigen::VectorXd& d_vec,
                       const cv::Mat& img, 
                       bool visualize=false);

} // namespace vo_utils 
} // namespace wolf

#endif /* PROCESSOR_VO_UTILS_H_ */
