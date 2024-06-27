//standard
#include "vision/processor/processor_visual_odometry.h"
#include "vision/processor/vo_utils.h"
#include "vision/math/pinhole_tools.h"

namespace wolf {
namespace vo_utils {


void retainBest(std::vector<cv::KeyPoint> &_keypoints, int n) {
    if (_keypoints.size() > n) {
        if (n == 0) {
            _keypoints.clear();
            return;
        }
        std::nth_element(_keypoints.begin(), _keypoints.begin() + n, _keypoints.end(),
            [](cv::KeyPoint& a, cv::KeyPoint& b) { return a.response > b.response; });
        _keypoints.resize(n);
    }
}


TracksMap mergeTracks(const TracksMap& tracks_prev_curr, const TracksMap& tracks_curr_next) {
    TracksMap tracks_prev_next;
    for (auto &match : tracks_prev_curr) {
        if (tracks_curr_next.count(match.second)) {
            tracks_prev_next[match.first] = tracks_curr_next.at(match.second);
        }
    }
    return tracks_prev_next;
}


TracksMap kltTrack(const wolf::ParamsProcessorVisualOdometryPtr _params_vo,
                   const cv::Mat& _img_prev, const cv::Mat& _img_curr,
                   const KeyPointsMap& _mwkps_prev, KeyPointsMap& _mwkps_curr) {
    ParamsProcessorVisualOdometry::KltParams prms = _params_vo->klt;

    if (_mwkps_prev.empty()) return TracksMap();

    TracksMap tracks_prev_curr;

    // Create cv point list for tracking, we initialize optical flow with previous keypoints
    // We also need a list of indices to build the track map
    std::vector<cv::Point2f> p2f_prev;
    std::vector<size_t> indices_prev;
    for (auto & wkp : _mwkps_prev) {
        p2f_prev.push_back(wkp.second.getCvKeyPoint().pt);
        indices_prev.push_back(wkp.first);
    }
    std::vector<cv::Point2f> p2f_curr = p2f_prev;

    // Configure and process KLT optical flow research
    std::vector<uchar> status;
    std::vector<float> err;



    // Process one way: previous->current with current init with previous
    cv::calcOpticalFlowPyrLK(
            _img_prev,
            _img_curr, 
            p2f_prev,
            p2f_curr,
            status, err,
            {prms.patch_width, prms.patch_height},
            prms.nlevels_pyramids,
            prms.criteria,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
    
    // Process the other way: current->previous
    std::vector<uchar> status_back;
    std::vector<float> err_back;
    cv::calcOpticalFlowPyrLK(
            _img_curr,
            _img_prev,
            p2f_curr,
            p2f_prev,
            status_back, err_back,
            {prms.patch_width, prms.patch_height},
            prms.nlevels_pyramids,
            prms.criteria,
            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));

    // Delete point if KLT failed
    for (size_t j = 0; j < status.size(); j++) {

        if(!status_back.at(j)  ||  (err_back.at(j) > prms.max_err) ||
           !status.at(j)  ||  (err.at(j) > prms.max_err)) {
            continue;
        }

        // We keep the initial point and add the tracked point
        WKeyPoint wkp(cv::KeyPoint(p2f_curr.at(j), 1));
        _mwkps_curr[wkp.getId()] = wkp;

        // Update the map
        tracks_prev_curr[indices_prev.at(j)] = wkp.getId();

        // Other checks? Distance between track points?
    }

    return tracks_prev_curr;
}


double getParallax(const Eigen::Vector4d& _pinhole_model, 
                   const KeyPointsMap& _mwkps_prev, const KeyPointsMap& _mwkps_curr, 
                   const TracksMap& _tracks_prev_curr) {
    // Parallax computation
    double avg_parallax = 0;
    for (const auto& track_prev_curr : _tracks_prev_curr) {
        Eigen::Vector2d p2d_prev = pinhole::depixellizePoint(_pinhole_model, _mwkps_prev.at(track_prev_curr.first).getEigenKeyPoint());
        Eigen::Vector2d p2d_curr = pinhole::depixellizePoint(_pinhole_model, _mwkps_curr.at(track_prev_curr.second).getEigenKeyPoint());

        Eigen::Vector3d ray_prev(p2d_prev(0), p2d_prev(1), 1.0);
        Eigen::Vector3d ray_curr(p2d_curr(0), p2d_curr(1), 1.0);

        avg_parallax += std::acos(ray_prev.normalized().transpose() *
                                  ray_curr.normalized());
    }
    avg_parallax /= _tracks_prev_curr.size();
    avg_parallax = avg_parallax * 180 / M_PI;
    
    return avg_parallax;
}


double evalReprojError(const std::vector<Eigen::Vector3d>& pts3d, 
                       const std::vector<Eigen::Vector2d>& pts2d,
                       const Eigen::Vector4d& K_vec,
                       const Eigen::VectorXd& d_vec,
                       const cv::Mat& img, 
                       bool visualize) {
    assert(pts3d.size() == pts2d.size());
    const size_t N = pts3d.size();
    
    cv::Mat img_vis = img.clone();
    if (img_vis.type() == CV_8UC1) {
        // Convert grayscale image (CV_8UC1) to BGR image (CV_8UC3)
        cv::cvtColor(img_vis, img_vis, cv::COLOR_GRAY2BGR);
    }

    double err = 0.0;

    for (size_t i=0; i<N; i++) {
        Eigen::Vector2d pt2d = pts2d.at(i);
        Eigen::Vector2d pt3d_proj = pinhole::projectPoint(K_vec, d_vec, pts3d.at(i));

        err += (pt2d - pt3d_proj).norm();
                    
        if (visualize) {
            cv::Point2d pt2d_cv(pt2d(0), pt2d(1));
            cv::Point2d pt3d_proj_cv(pt3d_proj(0), pt3d_proj(1));

            // Draw a circle around the keypoint
            cv::circle(img_vis, pt2d_cv, 2, cv::Scalar(255, 0, 0), 2);  // Blue circle with radius 2 (measured keypoint)
            cv::circle(img_vis, pt3d_proj_cv, 4, cv::Scalar(0, 0, 255), 2);  // Red circle with radius 4 (projected keypoint)
        }
    }
    err /= N;

    if (visualize) {
        std::cout << "reprojection error: " << err << " (total " << N << " pts.)" << std::endl;
        cv::imshow("Reprojection Visualiation", img_vis);
        cv::waitKey(-1);
    }
    
    return err;
}



} // namespace vo_utils
} // namespace wolf