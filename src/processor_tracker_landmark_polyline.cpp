#include "processor_tracker_landmark_polyline.h"

namespace wolf
{

void ProcessorTrackerLandmarkPolyline::preProcess()
{
    // extract corners of incoming
    extractPolylines((CaptureLaser2D*)((incoming_ptr_)), polylines_incoming_);

    // compute transformations
    Eigen::Vector3s vehicle_pose = getProblem()->getStateAtTimeStamp(incoming_ptr_->getTimeStamp());
    t_world_robot_ = vehicle_pose.head<2>();

    // world_robot
    Eigen::Matrix2s R_world_robot = Eigen::Rotation2Ds(vehicle_pose(2)).matrix();

    // robot_sensor (to be computed once if extrinsics are fixed and not dynamic)
    if (getSensorPtr()->isExtrinsicDynamic() || !getSensorPtr()->getPPtr()->isFixed()
            || !getSensorPtr()->getOPtr()->isFixed() || !extrinsics_transformation_computed_)
    {
        t_robot_sensor_ = getSensorPtr()->getPPtr()->getVector();
        R_robot_sensor_ = Eigen::Rotation2Ds(getSensorPtr()->getOPtr()->getVector()(0)).matrix();
        extrinsics_transformation_computed_ = true;
    }

    // global_sensor
    R_world_sensor_ = R_world_robot * R_robot_sensor_;
    t_world_sensor_ = t_world_robot_ + R_world_robot * t_robot_sensor_;

    // sensor_global
    R_sensor_world_ = R_robot_sensor_.transpose() * R_world_robot.transpose();
    t_sensor_world_ = -R_sensor_world_ * t_world_robot_ - R_robot_sensor_.transpose() * t_robot_sensor_;

    //std::cout << "t_robot_sensor_" << t_robot_sensor_.transpose() << std::endl;
    //std::cout << "t_world_robot_" << t_world_robot_.transpose() << std::endl;
    //std::cout << "t_world_sensor_" << t_world_sensor_.transpose() << std::endl;
    //std::cout << "PreProcess: incoming new features: " << corners_incoming_.size() << std::endl;
}

unsigned int ProcessorTrackerLandmarkPolyline::findLandmarks(const LandmarkBaseList& _landmarks_corner_searched,
                                                           FeatureBaseList& _features_corner_found,
                                                           LandmarkMatchMap& _feature_landmark_correspondences)
{
    //std::cout << "ProcessorTrackerLandmarkPolyline::findLandmarks: " << _landmarks_corner_searched.size() << " features: " << corners_incoming_.size()  << std::endl;

    // NAIVE FIRST NEAREST NEIGHBOR MATCHING
    LandmarkBaseList not_matched_landmarks = _landmarks_corner_searched;
    LandmarkPolylineMatch* best_match = nullptr;

    // COMPUTING ALL EXPECTED FEATURES
    std::map<LandmarkBase*, Eigen::MatrixXs> expected_features;
    std::map<LandmarkBase*, Eigen::MatrixXs> expected_features_covs;
    for (auto landmark : not_matched_landmarks)
        expectedFeature(landmark, expected_features[landmark], expected_features_covs[landmark]);

    FeaturePolyline2D* polyline_feature;
    LandmarkPolyline2D* polyline_landmark;

    auto next_feature_it = polylines_incoming_.begin();
    auto feature_it = next_feature_it++;
    int min_ftr, max_ftr, min_lmk, max_lmk, max_offset, min_offset, offset, from_ftr, from_lmk, N_overlapped;

    // iterate over all polylines features
    while (feature_it != polylines_incoming_.end())
    {
        polyline_feature = (FeaturePolyline2D*)(*feature_it);
        max_ftr = polyline_feature->getNPoints() - 1;

        // Check with all landmarks
        for (auto landmark_it = not_matched_landmarks.begin(); landmark_it != not_matched_landmarks.end(); landmark_it++)
        {
            polyline_landmark = (LandmarkPolyline2D*)(*landmark_it);

            // Check all overlapping positions between each feature-landmark pair
            max_lmk = polyline_landmark->getNPoints() - 1;
            max_offset = max_ftr - (polyline_landmark->isFirstExtreme() || polyline_feature->isLastExtreme() ? 1 : 0);
            min_offset = -max_lmk + (polyline_feature->isFirstExtreme() || polyline_landmark->isLastExtreme() ? 1 : 0);

            // Check all overlapping positions between each feature-landmark pair
            for (offset = min_offset; offset <= max_offset; offset++)
            {
                from_lmk = std::max(0, min_ftr-offset);
                from_ftr = std::max(0, min_lmk+offset);


                {

                }
                // B: landmark line (without detected points) from_lmk == 0 ||
                if (polyline_landmark->isFirstExtreme() && polyline_landmark->isLastExtreme() && polyline_landmark->getNPoints() == 2)
                {

                }
                // C. 2 polylines (at least one detected point in landmark and feature)
                else
                {
                    N_overlapped = std::min(max_ftr - from_ftr, max_lmk - from_lmk)+1;

                    // Compute the squared distance for all overlapped points
                    Eigen::ArrayXXd d = (polyline_feature->getPoints().block(0,from_ftr, 2,N_overlapped) -
                                         expected_features[*landmark_it].middleCols(from_lmk, N_overlapped)).array();

                    Eigen::ArrayXd dist2 = d.row(0).pow(2) + d.row(1).pow(2);

                    // First feature or landmark not extreme distance
                    bool ftr_first_not_extreme_overlapped = !polyline_feature->isFirstExtreme() && from_ftr == 0;
                    bool lmk_first_not_extreme_overlapped = !polyline_landmark->isFirstExtreme() && from_lmk == 0;
                    if (ftr_first_not_extreme_overlapped || lmk_first_not_extreme_overlapped)
                    {
                        //TODO: punt a línia from_ftr, from_ftr+1 i from_lmk
                        dist2(0) = distPoint2Line(polyline_feature->getPoints().col(from_ftr),
                                                  polyline_feature->getPoints().col(from_ftr+1),
                                                  expected_features[*landmark_it].col(from_lmk),
                                                  !ftr_first_not_extreme_overlapped,
                                                  !lmk_first_not_extreme_overlapped);
                    }
                    // Last feature or landmark not extreme distance
                    bool ftr_last_not_extreme_overlapped = !polyline_feature->isLastExtreme() && from_ftr+N_overlapped-1 == max_ftr;
                    bool lmk_last_not_extreme_overlapped = !polyline_landmark->isLastExtreme() && from_lmk+N_overlapped-1 == max_lmk;
                    if (ftr_last_not_extreme_overlapped || lmk_last_not_extreme_overlapped)
                    {
                        // punt a línia from_ftr, from_ftr+1 i from_lmk
                        dist2(N_overlapped-1) = distPoint2Line(polyline_feature->getPoints().col(from_ftr+N_overlapped-1),
                                                               polyline_feature->getPoints().col(from_ftr+N_overlapped-2),
                                                               expected_features[*landmark_it].col(from_lmk+N_overlapped-1),
                                                               !ftr_last_not_extreme_overlapped,
                                                               !lmk_last_not_extreme_overlapped);
                    }

                    // All squared distances should be witin a threshold
                    // Choose the most overlapped one
                    if ((dist2 < position_error_th_*position_error_th_).all() && (best_match == nullptr || N_overlapped > best_match->feature_match_to_id_-best_match->feature_match_from_id_+1))
                    {
                        best_match = new LandmarkPolylineMatch();
                        best_match->feature_match_from_id_= from_ftr;
                        best_match->landmark_match_from_id_= from_lmk;
                        best_match->feature_match_to_id_= from_ftr+N_overlapped-1;
                        best_match->landmark_match_to_id_= from_lmk+N_overlapped-1;
                        best_match->landmark_ptr_=*landmark_it;
                        best_match->normalized_score_ = dist2.mean();
                    }
                }
            }
        }
        // Match found for this feature
        if (best_match != nullptr)
        {
            //std::cout << "closest landmark: " << (*closest_landmark)->id() << std::endl;
            // match
            matches_landmark_from_incoming_[*feature_it] = best_match;
            // erase from the landmarks to be found
            not_matched_landmarks.remove(best_match->landmark_ptr_);
            // move corner feature to output list
            _features_corner_found.splice(_features_corner_found.end(), polylines_incoming_, feature_it);
            // reset match
            best_match = nullptr;
        }
        //else
        //    std::cout << "no landmark close enough found." << std::endl;
        feature_it = next_feature_it++;
    }
    return matches_landmark_from_incoming_.size();
}

bool ProcessorTrackerLandmarkPolyline::voteForKeyFrame()
{
    // option 1: more than TH new features in last
    if (polylines_last_.size() >= new_features_th_)
    {
        std::cout << "------------- NEW KEY FRAME: Option 1 - Enough new features" << std::endl;
        //std::cout << "\tnew features in last = " << corners_last_.size() << std::endl;
        return true;
    }
    // option 2: loop closure (if the newest frame from which a matched landmark was observed is old enough)
    for (auto new_ftr : new_features_last_)
    {
        if (last_ptr_->getFramePtr()->id() - matches_landmark_from_last_[new_ftr]->landmark_ptr_->getConstrainedByListPtr()->back()->getCapturePtr()->getFramePtr()->id() > loop_frames_th_)
        {
            std::cout << "------------- NEW KEY FRAME: Option 2 - Loop closure" << std::endl;
            //std::cout << "\tmatched landmark from frame = " << matches_landmark_from_last_[new_ftr].landmark_ptr_->getConstrainedByListPtr()->back()->getCapturePtr()->getFramePtr()->id() << std::endl;
            return true;
        }
    }
    //// option 3: less than half matched in origin, matched in incoming (more than half in last)
    //if (matches_landmark_from_incoming_.size()*2 < origin_ptr_->getFeatureListPtr()->size() && matches_landmark_from_last_.size()*2 > origin_ptr_->getFeatureListPtr()->size())
    //{
    //    std::cout << "------------- NEW KEY FRAME: Option 3 - " << std::endl;
    //    //std::cout << "\tmatches in incoming = " << matches_landmark_from_incoming_.size() << std::endl<< "\tmatches in origin = " << origin_ptr_->getFeatureListPtr()->size() << std::endl;
    //    return true;
    //}
    return false;
}

void ProcessorTrackerLandmarkPolyline::extractPolylines(CaptureLaser2D* _capture_laser_ptr,
                                                        FeatureBaseList& _polyline_list)
{
    // TODO: sort corners by bearing
    std::list<laserscanutils::Polyline> polylines;

    line_finder_.findPolylines(_capture_laser_ptr->getScan(), ((SensorLaser2D*)getSensorPtr())->getScanParams(), polylines);

    Eigen::Vector4s measurement;
    for (auto&& pl : polylines)
    {

        _polyline_list.push_back(new FeaturePolyline2D(pl.points_, pl.covs_, pl.is_first_extreme_, pl.is_last_extreme_));
        //std::cout << "new polyline detected: " << std::endl;
    }
}

void ProcessorTrackerLandmarkPolyline::expectedFeature(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& expected_feature_,
                                                   Eigen::MatrixXs& expected_feature_cov_)
{
    //std::cout << "ProcessorTrackerLandmarkPolyline::expectedFeature: " << std::endl;
    //std::cout << "Landmark global pose: " << _landmark_ptr->getPPtr()->getVector().transpose() << " " << _landmark_ptr->getOPtr()->getVector() << std::endl;
    //std::cout << "Robot global pose: " << t_world_robot_.transpose() << std::endl;
    //std::cout << "Sensor global pose: " << t_world_sensor_.transpose() << std::endl;
    LandmarkPolyline2D* polyline_landmark = (LandmarkPolyline2D*)_landmark_ptr;

    expected_feature_ = Eigen::MatrixXs::Zero(2,polyline_landmark->getNPoints());
    expected_feature_cov_ = Eigen::MatrixXs::Zero(2,2*polyline_landmark->getNPoints());
    for (auto i = 0; i < polyline_landmark->getPointStatePtrDeque().size(); i++)
    {
        // ------------ expected feature point
        expected_feature_.col(i) = R_sensor_world_ * polyline_landmark->getPointStatePtrDeque()[i]->getVector() - t_world_sensor_;
        //std::cout << "Expected point " << i << ": " << expected_feature_.col(i).transpose() << std::endl;

        // ------------ expected feature point covariance
        // TODO
        expected_feature_cov_.middleCols(i*2, 2) = Eigen::MatrixXs::Identity(2,2);
    }
}

Eigen::VectorXs ProcessorTrackerLandmarkPolyline::computeSquaredMahalanobisDistances(const Eigen::Vector2s& _feature,
                                                                                     const Eigen::Matrix2s& _feature_cov,
                                                                                     const Eigen::Vector2s& _expected_feature,
                                                                                     const Eigen::Matrix2s& _expected_feature_cov,
                                                                                     const Eigen::MatrixXs& _mu)
{
    // ------------------------ d
    Eigen::Vector2s d = _feature - _expected_feature;

    // ------------------------ Sigma_d
    Eigen::Matrix2s iSigma_d = (_feature_cov + _expected_feature_cov).inverse();
    Eigen::VectorXs squared_mahalanobis_distances(_mu.cols());
    for (unsigned int i = 0; i < _mu.cols(); i++)
    {
        squared_mahalanobis_distances(i) = (d - _mu.col(i)).transpose() * iSigma_d * (d - _mu.col(i));
        //if ((d - _mu.col(i)).norm() < 1)
        //{
        //    std::cout << "distance:    " << (d - _mu.col(i)).norm() << std::endl;
        //    std::cout << "iSigma_d:    " << std::endl << iSigma_d << std::endl;
        //    std::cout << "mahalanobis: " << squared_mahalanobis_distances(i) << std::endl;
        //}
    }

    return squared_mahalanobis_distances;
}

Scalar ProcessorTrackerLandmarkPolyline::distPointToLine(const Eigen::VectorXs& _A, const Eigen::VectorXs& _A_aux,
                                                         const Eigen::VectorXs& _B, bool _A_extreme, bool _B_extreme)
{
    if (!_A_extreme && !_B_extreme)
    {

    }
    else if (_A_extreme && !_B_extreme)
    {

    }
    else if (!_A_extreme && _B_extreme)
    {

    }

    // default return euclidean squared distance
    return (_A.head<2>() - _B.head<2>()).squaredNorm();
}

ProcessorBase* ProcessorTrackerLandmarkPolyline::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorParamsPolyline* params = (ProcessorParamsPolyline*)_params;
    ProcessorTrackerLandmarkPolyline* prc_ptr = new ProcessorTrackerLandmarkPolyline(params->line_finder_params_, params->new_corners_th, params->loop_frames_th);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

void ProcessorTrackerLandmarkPolyline::createNewLandmarks(LandmarkBaseList& _new_landmarks)
{
    FeaturePolyline2D* polyline_ft_ptr;
    LandmarkBase* new_lmk_ptr;
    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        new_lmk_ptr = createLandmark(new_feature_ptr);
        //std::cout << "\tnew_landmark: " << new_lmk_ptr->id() << std::endl;
        _new_landmarks.push_back(new_lmk_ptr);
        // cast
        polyline_ft_ptr = (FeaturePolyline2D*)(new_feature_ptr);
        // create new correspondence
        LandmarkPolylineMatch* match = new LandmarkPolylineMatch();
        match->feature_match_from_id_= 0; // all points match
        match->landmark_match_from_id_ = 0;
        match->normalized_score_ = 1.0; // max score
        matches_landmark_from_last_[new_feature_ptr] = match;
    }
}

LandmarkBase* ProcessorTrackerLandmarkPolyline::createLandmark(FeatureBase* _feature_ptr)
{
    //std::cout << "ProcessorTrackerLandmarkPolyline::createLandmark" << std::endl;
    FeaturePolyline2D* polyline_ptr = (FeaturePolyline2D*)(_feature_ptr);
    // compute feature global pose
    Eigen::MatrixXs points_global = R_world_sensor_ * polyline_ptr->getPoints().topRows<2>()
            + t_world_sensor_ * Eigen::VectorXs::Ones(polyline_ptr->getNPoints());
    // Create new landmark
    return new LandmarkPolyline2D(points_global, polyline_ptr->isFirstExtreme(), polyline_ptr->isLastExtreme());
}

ProcessorTrackerLandmarkPolyline::~ProcessorTrackerLandmarkPolyline()
{
    while (!polylines_last_.empty())
    {
        polylines_last_.front()->destruct();
        polylines_last_.pop_front();
    }
    while (!polylines_incoming_.empty())
    {
        polylines_incoming_.front()->destruct();
        polylines_incoming_.pop_front();
    }
}

void ProcessorTrackerLandmarkPolyline::establishConstraints()
{
    LandmarkPolylineMatch* polyline_match;
    FeaturePolyline2D* polyline_feature;
    LandmarkPolyline2D* polyline_landmark;
    ConstraintBase* new_constraint;

    for (auto last_feature : *(last_ptr_->getFeatureListPtr()))
    {
        polyline_feature = (FeaturePolyline2D*)last_feature;
        polyline_match = (LandmarkPolylineMatch*)matches_landmark_from_last_[last_feature];
        polyline_landmark = (LandmarkPolyline2D*)(matches_landmark_from_last_[last_feature]);

        Eigen::MatrixXs points_global = R_world_sensor_ * polyline_feature->getPoints().topRows<2>() +
                                        t_world_sensor_ * Eigen::VectorXs::Ones(polyline_feature->getNPoints());
        // GROW LANDMARK
        // Add front points
        if (polyline_match->feature_match_from_id_ > 0)
        {
            polyline_landmark->addPoints(points_global, // points matrix in global coordinates
                                         polyline_match->feature_match_from_id_-1, // last feature point index to be added
                                         polyline_feature->isFirstExtreme(), // is extreme
                                         false); // front

            polyline_match->feature_match_from_id_ = 0;
            polyline_match->landmark_match_to_id_ += polyline_match->feature_match_from_id_-1;
        }
        else if (polyline_feature->isFirstExtreme() && !polyline_landmark->isFirstExtreme())
        {
            polyline_landmark->setFirstExtreme(points_global.leftCols(1));
        }
        // Add back points
        if (polyline_match->feature_match_to_id_ < polyline_feature->getNPoints()-1)
        {
            polyline_landmark->addPoints(points_global, // points matrix in global coordinates
                                         polyline_match->feature_match_to_id_+1, // last feature point index to be added
                                         polyline_feature->isLastExtreme(), // is extreme
                                         true); // back

            polyline_match->feature_match_to_id_ = polyline_feature->getNPoints()-1;
            polyline_match->landmark_match_to_id_ += polyline_match->feature_match_to_id_-1;
        }
        else if (polyline_feature->isLastExtreme() && !polyline_landmark->isLastExtreme())
        {
            polyline_landmark->setLastExtreme(points_global.rightCols(1));
        }

        // ESTABLISH CONSTRAINTS
        // Constraints for all inner and extreme feature points
        unsigned int offset_id = polyline_match->landmark_match_from_id_ - polyline_match->feature_match_from_id_;
        for (unsigned int i = 0; i < polyline_feature->getNPoints(); i++)
        {
            if (i == 0 && polyline_feature->isFirstExtreme())
                continue;
            if (i == polyline_feature->getNPoints()-1 && polyline_feature->isLastExtreme())
                continue;

            // fill constraint
            last_feature->addConstraint(new ConstraintPoint2D(polyline_feature, polyline_landmark, i, i+offset_id));
        }
        // Constraints extreme lines (not enclosed by inner or extreme points)
        // a: only a line
        if (polyline_feature->isFirstExtreme() && polyline_feature->isLastExtreme() && polyline_feature->getNPoints() == 2)
        {
            StateBlock* landmark_state_block_1 = polyline_landmark->getPointStatePtrDeque()[offset_id];
            StateBlock* landmark_state_block_2 = polyline_landmark->getPointStatePtrDeque()[offset_id+1];
            // fill constraint
            // TODO: constraint line: new_constraint = new ConstraintLine(polyline_feature, 0, 1, landmark_state_block_1, landmark_state_block_2)

            // add constraint
            last_feature->addConstraint(new_constraint);
        }
        // b: a line in first
        else if (polyline_feature->isFirstExtreme())
        {
            StateBlock* landmark_state_block = polyline_landmark->getPointStatePtrDeque()[offset_id];
            // fill constraint
            // TODO: constraint line: new_constraint = new ConstraintLine(polyline_feature, 0, 1, landmark_state_block_1, landmark_state_block_2)

            // add constraint
            last_feature->addConstraint(new_constraint);
        }
        // c: a line in last
        else if (polyline_feature->isLastExtreme())
        {
            StateBlock* landmark_state_block = polyline_landmark->getPointStatePtrDeque()[polyline_feature->getNPoints()+offset_id-1];
            // fill constraint
            // TODO: constraint line: new_constraint = new ConstraintLine(CTR_LINE, polyline_feature, 0, 1, landmark_state_block_1, landmark_state_block_2)

            // add constraint
            last_feature->addConstraint(new_constraint);
        }
    }
}

ConstraintBase* ProcessorTrackerLandmarkPolyline::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
    // not used
    return nullptr;
}


}        //namespace wolf

// Register in the SensorFactory
#include "processor_factory.h"
//#include "factory.h"
namespace wolf {
namespace
{
const bool registered_prc_laser = ProcessorFactory::get().registerCreator("POLYLINE", ProcessorTrackerLandmarkPolyline::create);
}
} // namespace wolf
