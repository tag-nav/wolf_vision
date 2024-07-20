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


// Function to triangulate a 3D point from two 2D image points and camera poses
Eigen::Vector3d triangulate(const Eigen::Vector2d& pt2d_prev, 
                            const Eigen::Vector2d& pt2d_curr, 
                            const Eigen::Isometry3d& T_inC_ofW_prev, 
                            const Eigen::Isometry3d& T_inC_ofW_curr) 
{
    Eigen::Matrix4d A;

    // Convert Isometry3d to Matrix4d
    Eigen::Matrix4d T_prev = T_inC_ofW_prev.matrix();
    Eigen::Matrix4d T_curr = T_inC_ofW_curr.matrix();

    // Construct the A matrix for the Direct Linear Transformation (DLT) method
    A.row(0) = pt2d_prev(0) * T_prev.row(2) - T_prev.row(0);
    A.row(1) = pt2d_prev(1) * T_prev.row(2) - T_prev.row(1);
    A.row(2) = pt2d_curr(0) * T_curr.row(2) - T_curr.row(0);
    A.row(3) = pt2d_curr(1) * T_curr.row(2) - T_curr.row(1);

    // Solve for the homogeneous coordinates of the 3D point
    Eigen::Vector4d p_homogeneous = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
    
    // Convert from homogeneous coordinates to 3D coordinates
    Eigen::Vector3d p_inW = p_homogeneous.head<3>() / p_homogeneous(3);

    return p_inW;
}


Eigen::Isometry3d getRelativePoseByEpipolarGeometry(const std::vector<cv::Point2f>& pts_prev,
                                                    const std::vector<cv::Point2f>& pts_curr,
                                                    cv::Mat K,
                                                    const double scale)
{
    // Convert K to CV_64F if it is not already
    if (K.type() != CV_64F) {
        K.convertTo(K, CV_64F);
    }

    // Find the essential matrix using the given intrinsic camera matrix K
    cv::Mat E = cv::findEssentialMat(pts_prev, pts_curr, K);

    // Recover the relative pose (rotation and translation) from the essential matrix
    cv::Mat R_cv, t_cv;
    cv::recoverPose(E, pts_prev, pts_curr, K, R_cv, t_cv);

    // Convert rotation and translation from OpenCV to Eigen
    Eigen::Matrix3d R;
    cv::cv2eigen(R_cv, R);
    Eigen::Vector3d t;
    cv::cv2eigen(t_cv, t);

    // Normalize the translation vector to unit length
    t.normalize();

    // Scale the translation vector
    t *= scale;

    // Construct the relative transformation as an Isometry3d
    Eigen::Isometry3d T_inB_ofA = Eigen::Translation3d(t) * Eigen::Quaterniond(R);

    return T_inB_ofA;
}


Eigen::Isometry3d getTinW(const FrameBasePtr frame)
{
    Eigen::Isometry3d T_inW = Eigen::Translation3d(frame->getP()->getState()) 
                            * Eigen::Quaterniond(frame->getO()->getState().data());
    return T_inW;
}


void setTinW(const Eigen::Isometry3d& T_inW, FrameBasePtr frame)
{
    // Update the translation state of frame
    frame->getP()->setState(T_inW.translation());

    // Extract the rotation part as Quaterniond
    Eigen::Quaterniond q(T_inW.rotation());

    // Store quaternion components in Eigen::Vector4d
    Eigen::Vector4d q_vec;
    q_vec << q.x(), q.y(), q.z(), q.w();

    // Update the orientation state of frame
    frame->getO()->setState(q_vec);

    return;
}


void getFeaturePairs(const FrameBasePtr frame_prev, const FrameBasePtr frame_curr, 
                     const TrackMatrix& track_matrix, const SensorCameraPtr sen_cam,
                     const std::list<FeatureBasePtr>& features_curr,
                     std::vector<cv::Point2f>& pts_prev, std::vector<cv::Point2f>& pts_curr)
{
    pts_prev.clear();
    pts_curr.clear();

    // Retrieve 2D-2D feature matching pairs in between the frames
    for (const auto& e : features_curr)
    {
        auto feature_curr = std::dynamic_pointer_cast<const FeaturePointImage>(e);
        if (!feature_curr) continue;  // Skip if the cast fails

        // Retrieve the corresponding feature from the last frame
        auto feature_prev_base = track_matrix.feature(feature_curr->trackId(), frame_prev->getCaptureOf(sen_cam));
        auto feature_prev = std::dynamic_pointer_cast<const FeaturePointImage>(feature_prev_base);

        // Ensure the previous feature is not null
        assert(feature_prev != nullptr);

        // Get 2D keypoints associated with the features from the two frames
        Eigen::Vector2d pt2d_prev = feature_prev->getMeasurement();
        Eigen::Vector2d pt2d_curr = feature_curr->getMeasurement();

        pts_prev.push_back(cv::Point2f(pt2d_prev(0), pt2d_prev(1)));
        pts_curr.push_back(cv::Point2f(pt2d_curr(0), pt2d_curr(1)));
    }

    return;
}


cv::Mat getCameraProjectionMatrix(cv::Mat K, const Eigen::Isometry3d& T_inC_ofW)
{
    // Convert K to CV_64F if it is not already
    if (K.type() != CV_64F) {
        K.convertTo(K, CV_64F);
    }

    // Conversion from Eigen to cv::Mat<3,4>, where the left <3,3> is from rotation component of Eigen::Isometry3d and the right <3,1> is from translation
    cv::Mat T_inC_ofW_cv = cv::Mat::zeros(3, 4, CV_64F);

    // Fill the matrix with the rotation and translation components
    Eigen::Matrix3d R = T_inC_ofW.rotation();
    Eigen::Vector3d t = T_inC_ofW.translation();

    // Copy data from Eigen to OpenCV
    cv::eigen2cv(R, T_inC_ofW_cv(cv::Rect(0, 0, 3, 3))); // Copy rotation
    cv::eigen2cv(t, T_inC_ofW_cv(cv::Rect(3, 0, 1, 3))); // Copy translation

    // Matrix multiplication between cv::Mat<3,3> K and cv::Mat<3,4> T
    cv::Mat Cam = K * T_inC_ofW_cv;

    return Cam;
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
                       bool visualize,
                       const std::string fname) {
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
            cv::circle(img_vis, pt2d_cv, 4, cv::Scalar(0, 255, 0), 5);  // Blue circle with radius 2 (measured keypoint)
            cv::circle(img_vis, pt3d_proj_cv, 8, cv::Scalar(0, 0, 255), 3);  // Red circle with radius 4 (projected keypoint)
        }
    }
    err /= N;

    if (visualize) {
        std::cout << "reprojection error: " << err << " (total " << N << " pts.)" << std::endl;
        cv::imshow("Reprojection Visualiation", img_vis);
        cv::imwrite(fname, img_vis);
        cv::waitKey(-1);
    }
    
    return err;
}



} // namespace vo_utils
} // namespace wolf