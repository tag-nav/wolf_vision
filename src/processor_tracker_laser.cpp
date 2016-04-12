#include "processor_tracker_laser.h"

namespace wolf {

ProcessorTrackerLaser::ProcessorTrackerLaser::ProcessorTrackerLaser() :
        ProcessorTrackerFeature(PRC_TRACKER_LIDAR),
        scan_params_(((SensorLaser2D*)(upperNodePtr()))->getScanParams()),
        corner_alg_params_(((SensorLaser2D*)(upperNodePtr()))->getCornerAlgParams()),
        scan_incoming_(nullptr), scan_last_(nullptr)
{
}

ProcessorTrackerLaser::ProcessorTrackerLaser::~ProcessorTrackerLaser()
{

}

unsigned int ProcessorTrackerLaser::processKnown()
{
    /** \return The number of successful tracks.
     *
     * It operates on the \b incoming capture pointed by incoming_ptr_.
     *
     * This should do one of the following, depending on the design of the tracker -- see use_landmarks_:
     *   - Track Features against other Features in the \b origin Capture. Tips:
     *     - An intermediary step of matching against Features in the \b last Capture makes tracking easier.
     *     - Once tracked against last, then the link to Features in \b origin is provided by the Features' Constraints in \b last.
     *     - If required, correct the drift by re-comparing against the Features in origin.
     *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
     *   - Track Features against Landmarks in the Map. Tips:
     *     - The links to the Landmarks are in the Features' Constraints in last.
     *     - The Constraints in \b last need to be transferred to \b incoming (moved, not copied).
     *
     * The function must generate the necessary Features in the \b incoming Capture,
     * of the correct type, derived from FeatureBase.
     *
     * It must also generate the constraints, of the correct type, derived from ConstraintBase
     * (through ConstraintAnalytic or ConstraintSparse).
     */

    // EXTRACT ALL CORNERS
    std::list <laserscanutils::Corner> corners;

    laserscanutils::extractCorners(scan_params_, corner_alg_params_, scan_last_->getRanges(), corners);

    // TODO: Change to laserscanutils
    Eigen::Matrix4s cov_mat;
    Eigen::Matrix2s R = Eigen::Matrix2s::Identity();
    Eigen::Vector4s corner_measurement;

    // STORE CORNERS AS FEATURES
    for (auto corner_it = corners.begin(); corner_it != corners.end(); corner_it++)
    {
        // CORNER POSE (in sensor frame)
        corner_measurement.head(2) = corner_it->pt_.head(2);
        corner_measurement(2) = corner_it->orientation_;
        corner_measurement(3) = corner_it->aperture_;

        // CORNER POSE COVARIANCE (in sensor frame)
        // TODO: maybe in line object?
        WolfScalar L1 = corner_it->line_1_.length();
        WolfScalar L2 = corner_it->line_2_.length();
        WolfScalar cov_angle_line1 = 12 * corner_it->line_1_.error_ / (pow(L1,2) * (pow(corner_it->line_1_.np_,3)-pow(corner_it->line_1_.np_,2)));
        WolfScalar cov_angle_line2 = 12 * corner_it->line_2_.error_ / (pow(L2,2) * (pow(corner_it->line_2_.np_,3)-pow(corner_it->line_2_.np_,2)));


        //init cov in corner coordinates
        cov_mat << corner_it->line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0,
                   0, corner_it->line_2_.error_ + cov_angle_line2 * L2 * L2 / 4, 0, 0,
                   0, 0, cov_angle_line1 + cov_angle_line2, 0,
                   0, 0, 0, cov_angle_line1 + cov_angle_line2;
        cov_mat = 10*cov_mat;

        // Rotate covariance
        R = Eigen::Rotation2D<WolfScalar>(corner_it->orientation_).matrix();
        cov_mat.topLeftCorner<2,2>() = R.transpose() * cov_mat.topLeftCorner<2,2>() * R;

        // FEATURE CORNER 2D
        all_corners_incoming_.push_back(new FeatureCorner2D(corner_measurement, cov_mat));
    }

    // TRACK CORNERS
    unsigned int n_tracks = track(*(last_ptr_->getFeatureListPtr()), *(incoming_ptr_->getFeatureListPtr()));
    std::cout << "N of features tracked in incoming: " << n_tracks << std::endl;
    std::cout << "N of new features in incoming: " << getNewFeaturesListIncoming().size() << std::endl;

    return n_tracks;
}

unsigned int ProcessorTrackerLaser::detectNewFeatures()
{
    // TODO WHEN processKnownFeatures() DOESN'T PROCESS THE WHOLE SCAN
    return getNewFeaturesListIncoming().size();
}

unsigned int ProcessorTrackerLaser::track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out)
{
    // ITERATE OVER ALL DETECTED CORNERS
    auto corner_it = all_corners_incoming_.begin();
    while (corner_it != all_corners_incoming_.end())
    {
        // MATCHING WITH last FEATURES
        for (auto feature_last : _feature_list_in)
        {
            // CHECK MATCHING
            if (abs(feature_last->getMeasurement()(3) - (*corner_it)->getMeasurement(3)) < aperture_error_th_ &&                     // CHECK APERTURE
                (feature_last->getMeasurement().head(2) - (*corner_it)->getMeasurement().head(2)).squaredNorm() < position_error_th_ && // CHECK POSITION
                abs(feature_last->getMeasurement()(2) - (*corner_it)->getMeasurement(2)) < angular_error_th_)                        // CHECK ORIENTATION
            {
                // MATCH -> TRACKED
                _feature_list_out.push_back((*corner_it));
                corner_it = all_corners_incoming_.erase(corner_it);
                corner_it--;
            }
            else
            {
                // NO MATCH -> ADD NEW FEATURE
                addNewFeatureIncoming((*corner_it));
                corner_it = all_corners_incoming_.erase(corner_it);
                corner_it--;
            }
        }
    }

    return _feature_list_out.size();
}

LandmarkBase* ProcessorTrackerLaser::createLandmark(FeatureBase* _feature_ptr)
{
    //TODO: get the current pose of vehicle and
    return new LandmarkCorner2D(new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

ConstraintBase* ProcessorTrackerLaser::createConstraint(FeatureBase* _feature_ptr, NodeBase* _node_ptr)
{
    return nullptr;
}

bool ProcessorTrackerLaser::voteForKeyFrame()
{
    std::cout << "Ration origin/incomming thereshold: " << min_features_ratio_th_ << std::endl;
    std::cout << "origin features:   " << origin_ptr_->getFeatureListPtr()->size() << std::endl;
    std::cout << "incoming features: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;
    std::cout << "ratio:             " << (incoming_ptr_->getFeatureListPtr()->size() / origin_ptr_->getFeatureListPtr()->size())  << std::endl;
    if (incoming_ptr_->getFeatureListPtr()->size() / origin_ptr_->getFeatureListPtr()->size() < min_features_ratio_th_)
        std::cout << "VOTE for a new key frame " << std::endl;
    else
        std::cout << "DON'T VOTE for a new key frame " << std::endl;
    return (incoming_ptr_->getFeatureListPtr()->size() / origin_ptr_->getFeatureListPtr()->size() < min_features_ratio_th_);
}

} // namespace wolf
