#include "capture_laser_2D.h"

// unsigned int CaptureLaser2D::segment_window_size = 8;//window size to extract segments
// double CaptureLaser2D::theta_min = 0.4; //minimum theta between consecutive segments to detect corner. PI/8=0.39
// double CaptureLaser2D::theta_max_parallel = 0.1; //maximum theta between consecutive segments to fuse them in a single line.
// double CaptureLaser2D::k_sigmas = 3.;//How many std_dev are tolerated to count that a point is supporting a line
// unsigned int CaptureLaser2D::max_beam_distance = 5;//max number of beams of distance between lines to consider corner or concatenation
// double CaptureLaser2D::max_distance = 0.5;//max distance between line ends to consider corner or concatenation

//CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const std::vector<WolfScalar>& _ranges):
//	CaptureBase(_ts, _sensor_ptr, _ranges),
//	ranges_(data_.data(), _ranges.size()),
//	intensities_(data_.data(), 0)
//{
//    laser_ptr_ = (SensorLaser2D*)sensor_ptr_;
//}
CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, 
                               SensorBase* _sensor_ptr, 
                               const std::vector<float>& _ranges) 
                               :
                               CaptureBase(_ts, _sensor_ptr), 
                               ranges_(_ranges)
{
    laser_ptr_ = (SensorLaser2D*) sensor_ptr_;
}


//CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _intensities):
//    CaptureBase(_ts, _sensor_ptr, _ranges),
//    ranges_(data_.data(), _ranges.size()),
//    intensities_(data_.data(), _intensities.size())
//{
//      laser_ptr_ = (SensorLaser2D*)sensor_ptr_;
//}
CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, 
                               SensorBase* _sensor_ptr, 
                               const std::vector<float>& _ranges, 
                               const std::vector<float>& _intensities) 
                               :
                               CaptureBase(_ts, _sensor_ptr),
                               ranges_(_ranges), 
                               intensities_(_intensities)
{
    laser_ptr_ = (SensorLaser2D*) sensor_ptr_;
}


CaptureLaser2D::~CaptureLaser2D()
{
    //
}

void CaptureLaser2D::processCapture()
{
    //variables
    std::list <laserscanutils::Corner> corners;

    //extract corners from range data
    this->extractCorners(corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;

    //generate a feature for each corner
    this->createFeatures(corners);
    //std::cout << getFeatureListPtr()->size() << " Features created" << std::endl;

    //Establish constraints for each feature
    //establishConstraints();
    this->establishConstraintsMHTree();
    //std::cout << "Constraints created" << std::endl;
}

unsigned int CaptureLaser2D::extractCorners(std::list<laserscanutils::Corner> & _corner_list) const
{
//     std::list < laserscanutils::Line > line_list;
//     
//     laserscanutils::extractLines(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), ranges_, line_list);
//     return laserscanutils::extractCorners(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), line_list, _corner_list);
    return laserscanutils::extractCorners(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), ranges_, _corner_list);
}

unsigned int CaptureLaser2D::extractLines(std::list<laserscanutils::Line> & _line_list) const
{
    return laserscanutils::extractLines(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams().line_params_, ranges_, _line_list);
}

void CaptureLaser2D::createFeatures(std::list<laserscanutils::Corner> & _corner_list)
{
    // TODO: Sensor model
    Eigen::Matrix4s cov_mat;
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
    Eigen::Vector4s meas;

    //for each corner in the list create a feature
    for (auto corner_it = _corner_list.begin(); corner_it != _corner_list.end(); corner_it++)
    {
        meas.head(2) = corner_it->pt_.head(2);
        meas(2) = corner_it->orientation_;
        meas(3) = corner_it->aperture_;

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

        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << cov_mat << std::endl;

        // Rotate covariance
        R.topLeftCorner<2,2>() = Eigen::Rotation2D<WolfScalar>(corner_it->orientation_).matrix();
        cov_mat.topLeftCorner<3,3>() = R.transpose() * cov_mat.topLeftCorner<3,3>() * R;

        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << cov_mat << std::endl;

        //TODO: add the rest of descriptors and struct atributes
        this->addFeature((FeatureBase*) (new FeatureCorner2D(meas, 10*cov_mat)));
    }
}

void CaptureLaser2D::establishConstraintsMHTree()
{   
    if (getFeatureListPtr()->size() != 0)
    {
        //local declarations
        WolfScalar prob, dm2;
        unsigned int ii, jj;
        std::map<unsigned int, unsigned int> ft_lk_pairs;
        std::vector<bool> associated_mask;
        Eigen::MatrixXs corners(3,4);
        corners << -CONTAINER_LENGTH / 2, -CONTAINER_WIDTH / 2,  -CONTAINER_LENGTH / 2, -CONTAINER_WIDTH / 2,
                   -CONTAINER_WIDTH / 2,  -CONTAINER_LENGTH / 2, -CONTAINER_WIDTH / 2,  -CONTAINER_LENGTH / 2,
                    0,                     M_PI / 2,              M_PI,                 -M_PI / 2;
        Eigen::MatrixXs rotated_corners = corners;
        Eigen::VectorXs squared_mahalanobis_distances;

        // COMPUTING ALL EXPECTED FEATURES
        std::map<LandmarkBase*, Eigen::Vector4s> expected_features;
        std::map<LandmarkBase*, Eigen::Matrix3s> expected_features_covs;
        unsigned int i = 0;
        for (auto j_it = getTop()->getMapPtr()->getLandmarkListPtr()->begin(); j_it != getTop()->getMapPtr()->getLandmarkListPtr()->end(); j_it++, i++)
            computeExpectedFeature(*j_it, expected_features[*j_it], expected_features_covs[*j_it]);

        // SETTING ASSOCIATION TREE
        std::map<unsigned int, FeatureCorner2D*> features_map;
        std::map<unsigned int, LandmarkBase*> landmarks_map;
        std::map<unsigned int, unsigned int> landmarks_index_map;

        //tree object allocation and sizing
        AssociationTree tree;
        tree.resize( getFeatureListPtr()->size() , getTop()->getMapPtr()->getLandmarkListPtr()->size() );

        //set independent probabilities between feature-landmark pairs
        ii=0;
        for (auto i_it = getFeatureListPtr()->begin(); i_it != getFeatureListPtr()->end(); i_it++, ii++) //ii runs over extracted feature
        {
            features_map[ii] = (FeatureCorner2D*)(*i_it);
            //std::cout << "Feature: " << (*i_it)->nodeId() << std::endl << (*i_it)->getMeasurement().head(3).transpose() << std::endl;
            jj = 0;
            for (auto j_it = getTop()->getMapPtr()->getLandmarkListPtr()->begin(); j_it != getTop()->getMapPtr()->getLandmarkListPtr()->end(); j_it++, jj++)
            {
                if ((*j_it)->getType() == LANDMARK_CORNER)
                {
                    landmarks_map[jj] = (*j_it);
                    landmarks_index_map[jj] = 0;
                    //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << std::endl;
                    //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                    if (fabs(pi2pi(((FeatureCorner2D*)(*i_it))->getAperture() - (*j_it)->getDescriptor(0))) < MAX_ACCEPTED_APERTURE_DIFF)
                    {
                        dm2 = computeSquaredMahalanobisDistances(*i_it, expected_features[*j_it], expected_features_covs[*j_it], Eigen::MatrixXs::Zero(3,1))(0);//Mahalanobis squared
                        prob = (dm2 < 5*5 ? 5*erfc( sqrt(dm2/2) ) : 0); //prob = erfc( sqrt(dm2/2) ); //prob = erfc( sqrt(dm2)/1.4142136 );// sqrt(2) = 1.4142136
                        tree.setScore(ii,jj,prob);
                    }
                    else
                        tree.setScore(ii,jj,0.);//prob to 0
                }
                else if ((*j_it)->getType() == LANDMARK_CONTAINER)
                {
                    //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << " " << jj+1 << " " << jj+2 << " " << jj+3 << std::endl;
                    // Resize tree (expected 1 target for each landmark but containers have 4 targets)
                    if (i_it == getFeatureListPtr()->begin())
                        tree.resize( tree.numDetections() , tree.numTargets()+3 );

                    //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                    if (fabs(pi2pi(((FeatureCorner2D*)(*i_it))->getAperture() - 3 * M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF)
                    {
                        // Rotate corners (seen from feature coordinates)
                        rotated_corners.topRows(2) = Eigen::Rotation2D<WolfScalar>((*i_it)->getMeasurement()(2)).matrix() * corners.topRows(2);

                        squared_mahalanobis_distances = computeSquaredMahalanobisDistances((*i_it), expected_features[*j_it], expected_features_covs[*j_it], rotated_corners);

                        for (unsigned int c = 0; c < 4; c++, jj++)
                        {
                            landmarks_map[jj] = (*j_it);
                            landmarks_index_map[jj] = c;
                            prob = (squared_mahalanobis_distances(c) < 5.*5. ? 5*erfc( sqrt(squared_mahalanobis_distances(c)/2) ) : 0); //prob = erfc( sqrt(dm2/2) ); //prob = erfc( sqrt(dm2)/1.4142136 );// sqrt(2) = 1.4142136
                            tree.setScore(ii,jj,prob);
                        }
                        jj--;
                    }
                    else
                    {
                        for (unsigned int c = 0; c < 4; c++, jj++)
                        {
                            landmarks_map[jj] = (*j_it);
                            landmarks_index_map[jj] = c;
                            tree.setScore(ii,jj,0.);//prob to 0
                        }
                        jj--;
                    }
                }
                else
                    assert(true && "in association: landmark diferent from corner and container!");
            }
        }

        // Grows tree and make association pairs
        //std::cout << "solving tree" << std::endl;
        tree.solve(ft_lk_pairs,associated_mask);

        //print tree & score table
        //std::cout << "-------------" << std::endl;
        //tree.printTree();
        //tree.printScoreTable();

        // Vector of new landmarks to be created
        std::vector<FeatureCorner2D*> new_corner_landmarks(0);
        std::vector<std::pair<FeatureCorner2D*,LandmarkBase*>> new_container_landmarks(0);
        std::vector<std::pair<int,int>> new_container_indexs(0);

        // ESTABLISH PAIRS
        ii = 0; //runs over features
        for (auto i_it = getFeatureListPtr()->begin(); i_it != getFeatureListPtr()->end(); i_it++, ii++) //ii runs over extracted feature
        {
            if ( associated_mask[ii] == false ) //unassociated feature case
            {
                // TRY TO FIT A CONTAINER WITH ALL EXISTING LANDMARK CORNERS
                LandmarkCorner2D* old_corner_landmark_ptr = nullptr;
                int feature_idx, corner_idx;
                if (fitNewContainer((FeatureCorner2D*)(*i_it), old_corner_landmark_ptr, feature_idx, corner_idx))
                {
                    new_container_landmarks.push_back(std::pair<FeatureCorner2D*,LandmarkCorner2D*>((FeatureCorner2D*)(*i_it), old_corner_landmark_ptr));
                    new_container_indexs.push_back(std::pair<int,int>(feature_idx, corner_idx));
                }
                // CREATE A NEW LANDMARK CORNER
                else
                    new_corner_landmarks.push_back((FeatureCorner2D*)(*i_it));
            }
            else //feature-landmark association case
            {
                FeatureCorner2D* associed_feature = (FeatureCorner2D*)(*i_it);
                LandmarkBase* associed_landmark = landmarks_map[ft_lk_pairs[ii]];
                unsigned int associed_landmark_index = landmarks_index_map[ft_lk_pairs[ii]];

                if (associed_landmark->getType() == LANDMARK_CORNER)
                    associed_feature->addConstraint(new ConstraintCorner2DTheta(associed_feature,                      //feature pointer
                                                                               (LandmarkCorner2D*)(associed_landmark), //landmark pointer
                                                                               getFramePtr()->getPPtr(),               //_robotPPtr,
                                                                               getFramePtr()->getOPtr(),               //_robotOPtr,
                                                                               associed_landmark->getPPtr(),           //_landmarkPPtr,
                                                                               associed_landmark->getOPtr()));         //_landmarkOPtr,

                else if (associed_landmark->getType() == LANDMARK_CONTAINER)
                    associed_feature->addConstraint(new ConstraintContainer(associed_feature,                       //feature pointer
                                                                           (LandmarkContainer*)(associed_landmark), //landmark pointer
                                                                           associed_landmark_index,                 // corner index
                                                                           getFramePtr()->getPPtr(),                //_robotPPtr,
                                                                           getFramePtr()->getOPtr(),                //_robotOPtr,
                                                                           associed_landmark->getPPtr(),            //_landmarkPPtr,
                                                                           associed_landmark->getOPtr()));          //_landmarkOPtr,
            }
        }

        // CREATING NEW LANDMARKS
        Eigen::Vector3s feature_global_pose;

        // Global transformation
        Eigen::Vector2s t_robot = getFramePtr()->getPPtr()->getVector().head(2);
        Eigen::Matrix2s R_robot = getFramePtr()->getOPtr()->getRotationMatrix().topLeftCorner<2,2>();
        WolfScalar robot_orientation = getFramePtr()->getOPtr()->getYaw();

        // Sensor transformation
        Eigen::Vector2s t_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
        Eigen::Matrix2s R_sensor = getSensorPtr()->getOPtr()->getRotationMatrix().topLeftCorner<2,2>();
        WolfScalar sensor_orientation = getSensorPtr()->getOPtr()->getYaw();

        // NEW CORNERS
        for (auto it=new_corner_landmarks.begin(); it!=new_corner_landmarks.end();it++)
        {
            //get feature on sensor frame
            const Eigen::Vector2s& feature_position = (*it)->getMeasurement().head(2);
            const WolfScalar& feature_orientation = (*it)->getMeasurement()(2);

            //translate feature position and orientation to world (global)
            feature_global_pose.head(2) = R_robot * (R_sensor * feature_position + t_sensor) + t_robot;
            feature_global_pose(2) = pi2pi(feature_orientation + robot_orientation + sensor_orientation);

            createCornerLandmark(*it, feature_global_pose);
        }

        // NEW CONTAINERS
        for (unsigned int i = 0; i< new_container_landmarks.size() ;i++)
        {
            //get feature on sensor frame
            const Eigen::Vector2s& feature_position = new_container_landmarks.at(i).first->getMeasurement().head(2);
            const WolfScalar& feature_orientation = new_container_landmarks.at(i).first->getMeasurement()(2);

            //translate feature position and orientation to world (global)
            feature_global_pose.head(2) = R_robot * (R_sensor * feature_position + t_sensor) + t_robot;
            feature_global_pose(2) = pi2pi(feature_orientation + robot_orientation + sensor_orientation);

            createContainerLandmark(new_container_landmarks.at(i).first, feature_global_pose, (LandmarkCorner2D*)(new_container_landmarks.at(i).second), new_container_indexs.at(i).first, new_container_indexs.at(i).second);
        }
    }
}

Eigen::VectorXs CaptureLaser2D::computePrior(const TimeStamp& _now) const
{
    return Eigen::Vector3s(1, 2, 3);
}

void CaptureLaser2D::computeExpectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_, Eigen::Matrix3s& expected_feature_cov_)
{
    FrameBase* frame_ptr = getFramePtr();

    Eigen::Vector2s p_robot = frame_ptr->getPPtr()->getVector();
    WolfScalar      o_robot = frame_ptr->getOPtr()->getYaw();

    Eigen::Vector2s p_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
    WolfScalar      o_sensor = getSensorPtr()->getOPtr()->getYaw();
    Eigen::Matrix2s R_sensor = getSensorPtr()->getOPtr()->getRotationMatrix().topLeftCorner<2, 2>();

    Eigen::Vector2s p_landmark = _landmark_ptr->getPPtr()->getVector();
    WolfScalar      o_landmark = _landmark_ptr->getOPtr()->getYaw();

    WolfScalar o_rs = getSensorPtr()->getOPtr()->getYaw() + frame_ptr->getOPtr()->getYaw(); // Sum of theta_sensor and theta_robot
    Eigen::Matrix2s R_sr = Eigen::Rotation2D<WolfScalar>(o_rs).matrix(); // Sum of theta_sensor and theta_robot

    Eigen::Vector2s p_robot_landmark = p_robot - p_landmark;

    // ------------------------ Expected feature
    expected_feature_.head(2) = - R_sr.transpose() * p_robot_landmark - R_sensor.transpose() * p_sensor;
    expected_feature_(2) = pi2pi(o_landmark - o_robot - o_sensor);

    if (_landmark_ptr->getType() == LANDMARK_CONTAINER)
        expected_feature_(3) = 3 * M_PI / 2;
    else if (_landmark_ptr->getType() == LANDMARK_CORNER)
        expected_feature_(3) = ((LandmarkCorner2D*)(_landmark_ptr))->getAperture();

    // ------------------------ Sigma
    // JACOBIAN
    Eigen::Matrix<WolfScalar, 3, 6> Jlr = Eigen::Matrix<WolfScalar, 3, 6>::Zero();
    Jlr.block<2,2>(0,0) = -R_sr.transpose();
    Jlr(2,2) = -1;
    Jlr.block<2,2>(0,3) = R_sr.transpose();
    Jlr(0,2) = -p_robot_landmark(0)*sin(o_rs) + p_robot_landmark(1)*cos(o_rs);
    Jlr(1,2) = -p_robot_landmark(0)*cos(o_rs) - p_robot_landmark(1)*sin(o_rs);
    Jlr(2,2) = 1;

    // SIGMA (filling upper diagonal only)
    Eigen::MatrixXs Sigma = Eigen::MatrixXs::Zero(6,6);
    // Sigma_ll
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0,0);
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma, 0,2);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma, 2,2);
    // Sigma_lr
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 0,3);
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 0,5);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getPPtr(), Sigma, 2,3);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 2,5);
    // Sigma_rr
    getTop()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 3,3);
    getTop()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 3,5);
    getTop()->getCovarianceBlock(frame_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 5,5);

    expected_feature_cov_ = Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose();
}

Eigen::VectorXs CaptureLaser2D::computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const LandmarkBase* _landmark_ptr, const Eigen::MatrixXs& _mu)
{
    assert(_mu.rows() == 3 && "mahalanobis distance with bad number of mu components");

    FrameBase* frame_ptr = getFramePtr();

    Eigen::Vector2s p_robot = frame_ptr->getPPtr()->getVector();
    WolfScalar      o_robot = frame_ptr->getOPtr()->getYaw();

    Eigen::Vector2s p_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
    WolfScalar      o_sensor = getSensorPtr()->getOPtr()->getYaw();
    Eigen::Matrix2s R_sensor = getSensorPtr()->getOPtr()->getRotationMatrix().topLeftCorner<2, 2>();

    Eigen::Vector2s p_landmark = _landmark_ptr->getPPtr()->getVector();
    WolfScalar      o_landmark = _landmark_ptr->getOPtr()->getYaw();

    const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
    const WolfScalar&      o_feature = _feature_ptr->getMeasurement()(2);

    WolfScalar o_rs = getSensorPtr()->getOPtr()->getYaw() + frame_ptr->getOPtr()->getYaw(); // Sum of theta_sensor and theta_robot
    Eigen::Matrix2s R_sr = Eigen::Rotation2D<WolfScalar>(o_rs).matrix(); // Sum of theta_sensor and theta_robot
    Eigen::Vector2s p_robot_landmark = p_robot - p_landmark;

    // ------------------------ d
    Eigen::Vector3s d;
    d.head(2) = p_feature + R_sr.transpose()*p_robot_landmark + R_sensor.transpose()*p_sensor;
    d(2) = pi2pi(o_feature - o_landmark + o_robot + o_sensor);

//    std::cout << "robot = " << p_robot.transpose() << o_robot << std::endl;
//    std::cout << "sensor = " << p_sensor.transpose() << o_sensor << std::endl;
//    std::cout << "landmark = " << p_landmark.transpose() << o_landmark << std::endl;
//    std::cout << "feature = " << p_feature.transpose() << o_feature << std::endl;
//    std::cout << "robot - landmark = " << p_robot_landmark.transpose() << std::endl;
//    std::cout << "d = " << d.transpose() << std::endl;

    // ------------------------ Sigma_d
    // JACOBIAN
    Eigen::Matrix<WolfScalar, 3, 6> Jlr = Eigen::Matrix<WolfScalar, 3, 6>::Zero();
    Jlr.block<2,2>(0,0) = -R_sr.transpose();
    Jlr(2,2) = -1;
    Jlr.block<2,2>(0,3) = R_sr.transpose();
    Jlr(0,2) = -p_robot_landmark(0)*sin(o_rs) + p_robot_landmark(1)*cos(o_rs);
    Jlr(1,2) = -p_robot_landmark(0)*cos(o_rs) - p_robot_landmark(1)*sin(o_rs);
    Jlr(2,2) = 1;

    // SIGMA (filling upper diagonal only)
    const Eigen::Matrix3s& Sigma_feature = _feature_ptr->getMeasurementCovariance().topLeftCorner<3,3>();
    Eigen::MatrixXs Sigma = Eigen::MatrixXs::Zero(6,6);
    // Sigma_ll
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0,0);
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma, 0,2);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma, 2,2);
    // Sigma_lr
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 0,3);
    getTop()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 0,5);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getPPtr(), Sigma, 2,3);
    getTop()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 2,5);
    // Sigma_rr
    getTop()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 3,3);
    getTop()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 3,5);
    getTop()->getCovarianceBlock(frame_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 5,5);

    // MAHALANOBIS DISTANCES
    Eigen::Matrix3s iSigma_d = (Sigma_feature + Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose()).inverse();

    Eigen::VectorXs squared_mahalanobis_distances(_mu.cols());

    for (unsigned int i = 0; i < _mu.cols(); i++)
        squared_mahalanobis_distances(i) = (d - _mu.col(i)).transpose() * iSigma_d * (d - _mu.col(i));

    return squared_mahalanobis_distances;
}

Eigen::VectorXs CaptureLaser2D::computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const Eigen::Vector4s& _expected_feature, const Eigen::Matrix3s& _expected_feature_cov, const Eigen::MatrixXs& _mu)
{
    assert(_mu.rows() == 3 && "mahalanobis distance with bad number of mu components");

    const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
    const WolfScalar&      o_feature = _feature_ptr->getMeasurement()(2);

    // ------------------------ d
    Eigen::Vector3s d;
    d.head(2) = p_feature - _expected_feature.head(2);
    d(2) = pi2pi(o_feature - _expected_feature(2));

//    std::cout << "feature = " << p_feature.transpose() << o_feature << std::endl;
//    std::cout << "d = " << d.transpose() << std::endl;

    // ------------------------ Sigma_d
    Eigen::Matrix3s iSigma_d = (_feature_ptr->getMeasurementCovariance().topLeftCorner<3,3>() + _expected_feature_cov).inverse();

    Eigen::VectorXs squared_mahalanobis_distances(_mu.cols());

    for (unsigned int i = 0; i < _mu.cols(); i++)
        squared_mahalanobis_distances(i) = (d - _mu.col(i)).transpose() * iSigma_d * (d - _mu.col(i));

    return squared_mahalanobis_distances;
}

bool CaptureLaser2D::fitNewContainer(FeatureCorner2D* _corner_ptr, LandmarkCorner2D*& old_corner_landmark_ptr, int& feature_idx, int& corner_idx)
{
    //std::cout << "Trying container... aperture = " << _corner_ptr->getMeasurement()(3) << std::endl;

    // It has to be 90º corner feature
    if (std::fabs(pi2pi(_corner_ptr->getMeasurement()(3) + M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF)
    {
        // Check all existing corners searching a container
        for (auto landmark_it = getTop()->getMapPtr()->getLandmarkListPtr()->rbegin(); landmark_it != getTop()->getMapPtr()->getLandmarkListPtr()->rend(); landmark_it++)
        {
            if ((*landmark_it)->getType() == LANDMARK_CORNER // should be a corner
                    && std::fabs(pi2pi(((LandmarkCorner2D*)(*landmark_it))->getAperture() + M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF) // should be a corner
            {
                //std::cout << "landmark " << (*landmark_it)->nodeId() << std::endl;
                Eigen::MatrixXs corners_relative_positions(3,6);
                corners_relative_positions << CONTAINER_LENGTH, CONTAINER_LENGTH, 0,               CONTAINER_WIDTH, CONTAINER_WIDTH,  0,
                                              0,                CONTAINER_WIDTH,  CONTAINER_WIDTH, 0,               CONTAINER_LENGTH, CONTAINER_LENGTH,
                                              M_PI / 2,         M_PI,            -M_PI / 2,        M_PI / 2,        M_PI,            -M_PI / 2;

                Eigen::Matrix2s R_feature = Eigen::Rotation2D<WolfScalar>(_corner_ptr->getMeasurement()(2)).matrix();
                corners_relative_positions = - corners_relative_positions;
                corners_relative_positions.topRows(2) = R_feature * corners_relative_positions.topRows(2);

                Eigen::VectorXs squared_mahalanobis_distances = computeSquaredMahalanobisDistances(_corner_ptr, (*landmark_it), corners_relative_positions);

//                std::cout << "squared_mahalanobis_distances " << std::endl << squared_mahalanobis_distances << std::endl;
//                std::cout << "probabilities " << std::endl;
//                for (unsigned int i = 0; i < 6; i++)
//                    std::cout << erfc( sqrt(squared_mahalanobis_distances(i)/2) ) << std::endl;

                WolfScalar SMD_threshold = 8;
                if (squared_mahalanobis_distances(0) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(0)/2) ) > 0.8 )
                {
                    std::cout << "   large side! prob =  " << erfc( sqrt(squared_mahalanobis_distances(0)/2)) << std::endl;
                    feature_idx = 0;
                    corner_idx = 1;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(1) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(1)/2) ) > 0.8 )
                {
                    std::cout << "   diagonal!  prob = " << erfc( sqrt(squared_mahalanobis_distances(1)/2)) << std::endl;
                    feature_idx = 0;
                    corner_idx = 2;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(2) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(2)/2) ) > 0.8 )
                {
                    std::cout << "   short side!  prob = " << erfc( sqrt(squared_mahalanobis_distances(2)/2)) << std::endl;
                    feature_idx = 2;
                    corner_idx = 1;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(3) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(3)/2) ) > 0.8 )
                {
                    std::cout << "   short side! prob = " << erfc( sqrt(squared_mahalanobis_distances(3)/2)) << std::endl;
                    feature_idx = 1;
                    corner_idx = 2;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(4) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(4)/2) ) > 0.8 )
                {
                    std::cout << "   diagonal!  prob = " << erfc( sqrt(squared_mahalanobis_distances(4)/2)) << std::endl;
                    feature_idx = 1;
                    corner_idx = 3;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(5) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(5)/2) ) > 0.8 )
                {
                    std::cout << "   large side!  prob = " << erfc( sqrt(squared_mahalanobis_distances(5)/2)) << std::endl;
                    feature_idx = 1;
                    corner_idx = 0;
                    old_corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
            }
        }
    }
    return false;
}

void CaptureLaser2D::createCornerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose)
{
    //create new landmark at global coordinates
    StateBase* new_landmark_state_position = new StatePoint2D(getTop()->getNewStatePtr());
    getTop()->addState(new_landmark_state_position, _feature_global_pose.head(2));
    StateOrientation* new_landmark_state_orientation = new StateTheta(getTop()->getNewStatePtr());
    getTop()->addState(new_landmark_state_orientation, _feature_global_pose.tail(1));

    // Initialize landmark covariance
    Eigen::MatrixXs Sigma_robot = Eigen::MatrixXs::Zero(3,3);
    getTop()->getCovarianceBlock(getFramePtr()->getPPtr(), getFramePtr()->getPPtr(), Sigma_robot, 0,0);
    getTop()->getCovarianceBlock(getFramePtr()->getPPtr(), getFramePtr()->getOPtr(), Sigma_robot, 0,2);
    getTop()->getCovarianceBlock(getFramePtr()->getOPtr(), getFramePtr()->getOPtr(), Sigma_robot, 2,2);
    Sigma_robot.block<1,2>(2,0) = Sigma_robot.block<2,1>(0,2).transpose();

    Eigen::Matrix3s R_robot3D = getFramePtr()->getOPtr()->getRotationMatrix().matrix();
    Eigen::Matrix3s Sigma_landmark = Sigma_robot + R_robot3D.transpose() * _corner_ptr->getMeasurementCovariance().topLeftCorner<3,3>() * R_robot3D;

    getTop()->addCovarianceBlock(new_landmark_state_position, new_landmark_state_position, Sigma_landmark.topLeftCorner<2,2>());
    getTop()->addCovarianceBlock(new_landmark_state_position, (StateBase*)new_landmark_state_orientation, Sigma_landmark.block<2,1>(0,2));
    getTop()->addCovarianceBlock((StateBase*)new_landmark_state_orientation, (StateBase*)new_landmark_state_orientation, Sigma_landmark.block<1,1>(2,2));

    //add it to the slam map as a new landmark
    LandmarkCorner2D* new_landmark = new LandmarkCorner2D(new_landmark_state_position, new_landmark_state_orientation, _corner_ptr->getMeasurement()(3));
    _corner_ptr->addConstraint(new ConstraintCorner2DTheta(_corner_ptr,                //feature pointer
                                                           new_landmark,               //landmark pointer
                                                           getFramePtr()->getPPtr(),   //_robotPPtr,
                                                           getFramePtr()->getOPtr(),   //_robotOPtr,
                                                           new_landmark->getPPtr(),    //_landmarkPPtr,
                                                           new_landmark->getOPtr()));  //_landmarkOPtr,
    getTop()->getMapPtr()->addLandmark((LandmarkBase*)new_landmark);
}

void CaptureLaser2D::createContainerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose, LandmarkCorner2D* _old_corner_landmark_ptr, int& _feature_idx, int& _corner_idx)
{
    assert(_old_corner_landmark_ptr != nullptr && "fitting result = true but corner found is nullptr");

    // CREATING LANDMARK CONTAINER
    // create new landmark state units
    StateBase* new_container_position = new StatePoint2D(getTop()->getNewStatePtr());
    getTop()->addState(new_container_position, Eigen::Vector2s::Zero());
    StateOrientation* new_container_orientation = new StateTheta(getTop()->getNewStatePtr());
    getTop()->addState(new_container_orientation, Eigen::Vector1s::Zero());

    // create new landmark
    Eigen::Vector3s corner_pose;
    corner_pose.head(2) = _old_corner_landmark_ptr->getPPtr()->getVector();
    corner_pose(2) = _old_corner_landmark_ptr->getOPtr()->getYaw();

    LandmarkContainer* new_landmark = new LandmarkContainer(new_container_position, new_container_orientation, _feature_global_pose, corner_pose, _feature_idx, _corner_idx, CONTAINER_WIDTH, CONTAINER_LENGTH);

    // add new landmark in the map
    getTop()->getMapPtr()->addLandmark((LandmarkBase*)new_landmark);

    // initialize container covariance with landmark corner covariance
    Eigen::MatrixXs Sigma_landmark = Eigen::MatrixXs::Zero(3,3);
    getTop()->getCovarianceBlock(_old_corner_landmark_ptr->getPPtr(), _old_corner_landmark_ptr->getPPtr(), Sigma_landmark, 0,0);
    getTop()->getCovarianceBlock(_old_corner_landmark_ptr->getPPtr(), _old_corner_landmark_ptr->getOPtr(), Sigma_landmark, 0,2);
    getTop()->getCovarianceBlock(_old_corner_landmark_ptr->getOPtr(), _old_corner_landmark_ptr->getOPtr(), Sigma_landmark, 2,2);
    Sigma_landmark.block<1,2>(2,0) = Sigma_landmark.block<2,1>(0,2).transpose();

    getTop()->addCovarianceBlock(new_container_position, new_container_position, Sigma_landmark.topLeftCorner<2,2>());
    getTop()->addCovarianceBlock(new_container_position, (StateBase*)new_container_orientation, Sigma_landmark.block<2,1>(0,2));
    getTop()->addCovarianceBlock((StateBase*)new_container_orientation, (StateBase*)new_container_orientation, Sigma_landmark.block<1,1>(2,2));

    // create new constraint (feature to container)
    _corner_ptr->addConstraint(new ConstraintContainer(_corner_ptr,               //feature pointer
                                                       new_landmark,              //landmark pointer
                                                       _feature_idx,              //corner idx
                                                       getFramePtr()->getPPtr(),  //_robotPPtr,
                                                       getFramePtr()->getOPtr(),  //_robotOPtr,
                                                       new_landmark->getPPtr(),   //_landmarkPPtr,
                                                       new_landmark->getOPtr())); //_landmarkOPtr,


    // ERASING LANDMARK CORNER
    // change all constraints from corner landmark by new corner container
    for (auto ctr_it = _old_corner_landmark_ptr->getConstraints()->begin(); ctr_it != _old_corner_landmark_ptr->getConstraints()->end(); ctr_it++)
    {
        // create new constraint to landmark container
        (*ctr_it)->getFeaturePtr()->addConstraint(new ConstraintContainer((*ctr_it)->getFeaturePtr(),//feature pointer
                                                                          new_landmark,              //landmark pointer
                                                                          _corner_idx,                //corner idx
                                                                          (*ctr_it)->getStatePtrVector().at(0),  //_robotPPtr,
                                                                          (StateOrientation*)(*ctr_it)->getStatePtrVector().at(1),  //_robotOPtr,
                                                                          new_landmark->getPPtr(),   //_landmarkPPtr,
                                                                          new_landmark->getOPtr())); //_landmarkOPtr,
    }
    // Remove corner landmark (it will remove all old constraints)
    getTop()->getMapPtr()->removeLandmark(_old_corner_landmark_ptr);
}
