#include "processor_tracker_landmark_corner.h"

namespace wolf
{

void ProcessorTrackerLandmarkCorner::preProcess()
{
    // extract corners of incoming
    extractCorners((CaptureLaser2D*)((incoming_ptr_)), corners_incoming_);

    // compute transformations
    t_world_robot_ = getProblem()->getStateAtTimeStamp(incoming_ptr_->getTimeStamp());

    // world_robot
    Eigen::Matrix3s R_world_robot = Eigen::Matrix3s::Identity();
    R_world_robot.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(t_world_robot_(2)).matrix();

    // robot_sensor (to be computed once if extrinsics are fixed and not dynamic)
    if (getSensorPtr()->isExtrinsicDynamic() || !getSensorPtr()->getPPtr()->isFixed()
            || !getSensorPtr()->getOPtr()->isFixed() || !extrinsics_transformation_computed_)
    {
        t_robot_sensor_.head<2>() = getSensorPtr()->getPPtr()->getVector();
        t_robot_sensor_(2) = getSensorPtr()->getOPtr()->getVector()(0);
        R_robot_sensor_.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(t_robot_sensor_(2)).matrix();
        extrinsics_transformation_computed_ = true;
    }

    // global_sensor
    R_world_sensor_.topLeftCorner<2, 2>() = R_world_robot.topLeftCorner<2, 2>() * R_robot_sensor_.topLeftCorner<2, 2>();
    t_world_sensor_ = t_world_robot_ + R_world_robot * t_robot_sensor_;

    // sensor_global
    R_sensor_world_.topLeftCorner<2, 2>() = R_robot_sensor_.topLeftCorner<2, 2>().transpose() * R_world_robot.topLeftCorner<2, 2>().transpose();
    t_sensor_world_ = -R_sensor_world_ * t_world_robot_ - R_robot_sensor_.transpose() * t_robot_sensor_;

    //std::cout << "t_robot_sensor_" << t_robot_sensor_.transpose() << std::endl;
    //std::cout << "t_world_robot_" << t_world_robot_.transpose() << std::endl;
    //std::cout << "t_world_sensor_" << t_world_sensor_.transpose() << std::endl;
    //std::cout << "PreProcess: incoming new features: " << corners_incoming_.size() << std::endl;
}

unsigned int ProcessorTrackerLandmarkCorner::findLandmarks(const LandmarkBaseList& _landmarks_corner_searched,
                                                           FeatureBaseList& _features_corner_found,
                                                           LandmarkMatchMap& _feature_landmark_correspondences)
{
    std::cout << "ProcessorTrackerLandmarkCorner::findLandmarks: " << _landmarks_corner_searched.size() << " features: " << corners_incoming_.size()  << std::endl;

    // NAIVE FIRST NEAREST NEIGHBOR MATCHING
    LandmarkBaseList not_matched_landmarks = _landmarks_corner_searched;
    Scalar dm2;

    // COMPUTING ALL EXPECTED FEATURES
    std::map<LandmarkBase*, Eigen::Vector4s> expected_features;
    std::map<LandmarkBase*, Eigen::Matrix3s> expected_features_covs;
    for (auto landmark : not_matched_landmarks)
        expectedFeature(landmark, expected_features[landmark], expected_features_covs[landmark]);

    auto next_feature_it = corners_incoming_.begin();
    auto feature_it = next_feature_it++;
    while (feature_it != corners_incoming_.end())
    {
        LandmarkBaseIter closest_landmark = not_matched_landmarks.end();
        Scalar closest_dm2 = 1e3;
        for (auto landmark_it = not_matched_landmarks.begin(); landmark_it != not_matched_landmarks.end(); landmark_it++)
        {
            if ((*landmark_it)->getType() == LANDMARK_CORNER &&
                fabs(pi2pi(((FeatureCorner2D*)(*feature_it))->getAperture() - (*landmark_it)->getDescriptor(0))) < aperture_error_th_)
            {
                dm2 = computeSquaredMahalanobisDistances((*feature_it), expected_features[*landmark_it],
                                                         expected_features_covs[*landmark_it],
                                                         Eigen::MatrixXs::Zero(3, 1))(0); //Mahalanobis squared
                if (dm2 < 0.5 && (closest_landmark == not_matched_landmarks.end() || dm2 < closest_dm2))
                {
                    //std::cout << "pair feature " << (*feature_it)->id() << " & landmark " << (*landmark_it)->id() << std::endl;
                    //std::cout << "pair SqMahalanobisDist = " << dm2 << std::endl;
                    closest_dm2 = dm2;
                    closest_landmark = landmark_it;
                }
            }
        }
        //std::cout << "all landmarks checked with feature " << (*feature_it)->id() << std::endl;
        if (closest_landmark != not_matched_landmarks.end())
        {
            //std::cout << "closest landmark: " << (*closest_landmark)->id() << std::endl;
            // match
            matches_landmark_from_incoming_[*feature_it] = LandmarkMatch({*closest_landmark, closest_dm2});
            // erase from the landmarks to be found
            not_matched_landmarks.erase(closest_landmark);
            // move corner feature to output list
            _features_corner_found.splice(_features_corner_found.end(), corners_incoming_, feature_it);
        }
        //else
        //    std::cout << "no landmark close enough found." << std::endl;
        feature_it = next_feature_it++;
    }

/*
    // MATCHING FROM MAP
    if (!corners_incoming_.empty() && !_landmarks_corner_searched.empty())
    {
        //local declarations
        Scalar prob, dm2;
        unsigned int ii, jj;

        // COMPUTING ALL EXPECTED FEATURES
        std::map<LandmarkBase*, Eigen::Vector4s> expected_features;
        std::map<LandmarkBase*, Eigen::Matrix3s> expected_features_covs;
        for (auto landmark : _landmarks_corner_searched)
            expectedFeature(landmark, expected_features[landmark], expected_features_covs[landmark]);
        //std::cout << "expected features!" << std::endl;

        // SETTING ASSOCIATION TREE
        std::map<unsigned int, FeatureBaseIter> features_map;
        std::map<unsigned int, LandmarkBaseIter> landmarks_map;
        std::map<unsigned int, unsigned int> landmarks_index_map;
        //tree object allocation and sizing
        AssociationTree tree;
        tree.resize(corners_incoming_.size(), _landmarks_corner_searched.size());
        //set independent probabilities between feature-landmark pairs
        ii = 0;
        for (auto feature_it = corners_incoming_.begin(); feature_it != corners_incoming_.end();
                feature_it++, ii++)    //ii runs over extracted feature
        {
            features_map[ii] = feature_it;
            //std::cout << "Feature: " << (*i_it)->nodeId() << std::endl << (*i_it)->getMeasurement().head(3).transpose() << std::endl;
            jj = 0;
            for (auto landmark_it = _landmarks_corner_searched.begin(); landmark_it != _landmarks_corner_searched.end();
                    landmark_it++, jj++)
            {
                if ((*landmark_it)->getType() == LANDMARK_CORNER)
                {
                    landmarks_map[jj] = landmark_it;
                    landmarks_index_map[jj] = 0;
                    //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << std::endl;
                    //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                    if (fabs(
                            pi2pi(((FeatureCorner2D*)(((*feature_it))))->getAperture()
                                    - (*landmark_it)->getDescriptor(0))) < aperture_error_th_)
                    {
                        dm2 = computeSquaredMahalanobisDistances(*feature_it, expected_features[*landmark_it],
                                                                 expected_features_covs[*landmark_it],
                                                                 Eigen::MatrixXs::Zero(3, 1))(0); //Mahalanobis squared
                        //prob = (dm2 < 5 * 5 ? 5 * erfc(sqrt(dm2 / 2)) : 0); //prob = erfc( sqrt(dm2/2) ); //prob = erfc( sqrt(dm2)/1.4142136 );// sqrt(2) = 1.4142136
                        prob = 10*erfc( sqrt(dm2/2) );
                        if (dm2 < 0.5)
                        {
                            std::cout << "pair feature " << (*features_map[ii])->id() << " & landmark " << (*landmarks_map[jj])->id() << std::endl;
                            std::cout << "pair SqMahalanobisDist = " << dm2 << std::endl;
                            std::cout << "pair probability = " << prob << std::endl;

                        }
                        tree.setScore(ii, jj, prob);
                    }
                    else
                        tree.setScore(ii, jj, 0.); //prob to 0
                }
            }
        }
        // Grows tree and make association pairs
        std::map<unsigned int, unsigned int> ft_lk_pairs;
        std::vector<bool> associated_mask;
        //std::cout << "solving tree" << std::endl;
        tree.solve(ft_lk_pairs, associated_mask);
        //print tree & score table
        //std::cout << "------------- TREE SOLVED ---------" << std::endl;
        //std::cout << corners_incoming_.size() << " new corners in incoming:" << std::endl;
        //for (auto new_feature : corners_incoming_)
        //    std::cout << "\tcorner " << new_feature->id() << std::endl;
        std::cout << ft_lk_pairs.size() << " pairs:" << std::endl;
        for (auto pair : ft_lk_pairs)
            std::cout << "\tfeature " << (*features_map[pair.first])->id() << " & landmark " << (*landmarks_map[pair.second])->id() << std::endl;
        //tree.printTree();
        //tree.printScoreTable();
        // Vector of new landmarks to be created
        std::vector<FeatureCorner2D*> new_corner_landmarks(0);

        // ESTABLISH CORRESPONDENCES
        for (auto pair : ft_lk_pairs)
        {
            // match
            matches_landmark_from_incoming_[*features_map[pair.first]] = LandmarkMatch(
                    *landmarks_map[pair.second], tree.getScore(pair.first, pair.second));
            // move matched feature to list
            _features_corner_found.splice(_features_corner_found.end(), corners_incoming_, features_map[pair.first]);
        }
        //std::cout << corners_incoming_.size() << " remaining new corners in incoming:" << std::endl;
        //for (auto new_feature : corners_incoming_)
        //    std::cout << "\tcorner " << new_feature->id() << std::endl;
    }
    else
        std::cout << "0 corners incoming or 0 landmarks to search" <<std::endl;
*/
    return matches_landmark_from_incoming_.size();
}

bool ProcessorTrackerLandmarkCorner::voteForKeyFrame()
{
    return matches_landmark_from_incoming_.size() < n_corners_th_ && matches_landmark_from_last_.size() > matches_landmark_from_incoming_.size();
}

void ProcessorTrackerLandmarkCorner::extractCorners(CaptureLaser2D* _capture_laser_ptr,
                                                    FeatureBaseList& _corner_list)
{
    // TODO: sort corners by bearing
    std::list<laserscanutils::CornerPoint> corners;

    corner_finder_.findCorners(_capture_laser_ptr->getScan(), ((SensorLaser2D*)getSensorPtr())->getScanParams(), line_finder_, corners);

    Eigen::Vector4s measurement;
    for (auto corner : corners)
    {
        measurement.head<2>() = corner.point_.head<2>();
        measurement(2)=corner.orientation_;
        measurement(3)=corner.aperture_;

        _corner_list.push_back(new FeatureCorner2D(measurement, corner.covariance_));

        //std::cout << "new corner detected: " << measurement.head<3>().transpose() << std::endl;
    }

/*    //variables
    std::list<laserscanutils::Corner> corners;
    //extract corners from range data
    laserscanutils::extractCorners(scan_params_, corner_alg_params_, _capture_laser_ptr->getRanges(), corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;
    Eigen::Matrix4s measurement_cov;
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
    Eigen::Vector4s measurement;
    for (auto corner : corners)
    {
        measurement.head(2) = corner.pt_.head(2);
        measurement(2) = corner.orientation_;
        measurement(3) = corner.aperture_;
        // TODO: maybe in line object?
        Scalar L1 = corner.line_1_.length();
        Scalar L2 = corner.line_2_.length();
        Scalar cov_angle_line1 = 12 * corner.line_1_.error_
                / (pow(L1, 2) * (pow(corner.line_1_.np_, 3) - pow(corner.line_1_.np_, 2)));
        Scalar cov_angle_line2 = 12 * corner.line_2_.error_
                / (pow(L2, 2) * (pow(corner.line_2_.np_, 3) - pow(corner.line_2_.np_, 2)));
        //init cov in corner coordinates
        measurement_cov << corner.line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0, 0, corner.line_2_.error_
                + cov_angle_line2 * L2 * L2 / 4, 0, 0, 0, 0, cov_angle_line1 + cov_angle_line2, 0, 0, 0, 0, cov_angle_line1
                + cov_angle_line2;
        measurement_cov = 10 * measurement_cov;
        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << measurement_cov << std::endl;
        // Rotate covariance
        R.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(corner.orientation_).matrix();
        measurement_cov.topLeftCorner<3, 3>() = R.transpose() * measurement_cov.topLeftCorner<3, 3>() * R;
        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << measurement_cov << std::endl;
        _corner_list.push_back(new FeatureCorner2D(measurement, measurement_cov));
    }*/
}

void ProcessorTrackerLandmarkCorner::expectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_,
                                                   Eigen::Matrix3s& expected_feature_cov_)
{
    //std::cout << "ProcessorTrackerLandmarkCorner::expectedFeature: " << std::endl;
    //std::cout << "Landmark global pose: " << _landmark_ptr->getPPtr()->getVector().transpose() << " " << _landmark_ptr->getOPtr()->getVector() << std::endl;
    //std::cout << "Robot global pose: " << t_world_robot_.transpose() << std::endl;
    //std::cout << "Sensor global pose: " << t_world_sensor_.transpose() << std::endl;

    // ------------ expected feature measurement
    expected_feature_.head<2>() = R_sensor_world_.topLeftCorner<2,2>() * (_landmark_ptr->getPPtr()->getVector() - t_world_sensor_.head<2>());
    expected_feature_(2) = _landmark_ptr->getOPtr()->getVector()(0) - t_world_sensor_(2);
    expected_feature_(3) = ((LandmarkCorner2D*)((_landmark_ptr)))->getAperture();
    //std::cout << "Expected feature pose: " << expected_feature_.head<3>().transpose() << std::endl;

    // ------------ expected feature covariance
    Eigen::MatrixXs Sigma = Eigen::MatrixXs::Zero(6, 6);
    // closer keyframe with computed covariance
    FrameBase* key_frame_ptr = (origin_ptr_ != nullptr ? origin_ptr_->getFramePtr() : nullptr);
    // If all covariance blocks are stored wolfproblem (filling upper diagonal only)
    if (key_frame_ptr != nullptr &&
        // Sigma_rr
        getProblem()->getCovarianceBlock(key_frame_ptr->getPPtr(), key_frame_ptr->getPPtr(), Sigma, 3, 3) &&
        getProblem()->getCovarianceBlock(key_frame_ptr->getPPtr(), key_frame_ptr->getOPtr(), Sigma, 3, 5) &&
        getProblem()->getCovarianceBlock(key_frame_ptr->getOPtr(), key_frame_ptr->getOPtr(), Sigma, 5, 5) &&
        // Sigma_ll
        getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0, 0) &&
        getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma, 0, 2) &&
        getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma, 2, 2) &&
        // Sigma_lr
        getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), key_frame_ptr->getPPtr(), Sigma, 0, 3) &&
        getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), key_frame_ptr->getOPtr(), Sigma, 0, 5) &&
        getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), key_frame_ptr->getPPtr(), Sigma, 2, 3) &&
        getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), key_frame_ptr->getOPtr(), Sigma, 2, 5) )
    {
        // Jacobian
        Eigen::Vector2s p_robot_landmark = t_world_robot_.head<2>() - _landmark_ptr->getPPtr()->getVector();
        Eigen::Matrix<Scalar, 3, 6> Jlr = Eigen::Matrix<Scalar, 3, 6>::Zero();
        Jlr.block<3, 3>(0, 3) = -R_world_sensor_.transpose();
        Jlr.block<3, 3>(0, 3) = R_world_sensor_.transpose();
        Jlr(0, 2) = -p_robot_landmark(0) * sin(t_world_sensor_(2)) + p_robot_landmark(1) * cos(t_world_sensor_(2));
        Jlr(1, 2) = -p_robot_landmark(0) * cos(t_world_sensor_(2)) - p_robot_landmark(1) * sin(t_world_sensor_(2));
        // measurement covariance
        expected_feature_cov_ = Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose();
    }
    else
        // Any covariance block is not stored in wolfproblem -> Identity()
        expected_feature_cov_ = Eigen::Matrix3s::Identity()*0.1;
}

Eigen::VectorXs ProcessorTrackerLandmarkCorner::computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr,
                                                                          const Eigen::Vector4s& _expected_feature,
                                                                          const Eigen::Matrix3s& _expected_feature_cov,
                                                                          const Eigen::MatrixXs& _mu)
{

    const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
    const Scalar& o_feature = _feature_ptr->getMeasurement()(2);
    // ------------------------ d
    Eigen::Vector3s d;
    d.head(2) = p_feature - _expected_feature.head(2);
    d(2) = pi2pi(o_feature - _expected_feature(2));
    // ------------------------ Sigma_d
    Eigen::Matrix3s iSigma_d =
            (_feature_ptr->getMeasurementCovariance().topLeftCorner<3, 3>() + _expected_feature_cov).inverse();
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

}        //namespace wolf
