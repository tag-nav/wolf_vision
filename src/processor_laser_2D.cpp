#include "processor_laser_2D.h"

namespace wolf {

ProcessorLaser2D::ProcessorLaser2D() : ProcessorBase(PRC_LIDAR),
        //sensor_laser_ptr_((SensorLaser2D*)(upperNodePtr())), // Static cast to specific sensor at construction time TODO: in construction time upperNodePtr is nullptr, it crashes always, to be removed or changed to somewhere (JVN)
		sensor_laser_ptr_(nullptr),
        capture_laser_ptr_(nullptr)
{
    setType("LASER 2D");
}

ProcessorLaser2D::~ProcessorLaser2D()
{
}

void ProcessorLaser2D::process(CaptureBase* _capture_ptr)
{
    extractFeatures(_capture_ptr);
    establishConstraints(_capture_ptr);
}

void ProcessorLaser2D::extractFeatures(CaptureBase* _capture_ptr)
{
    // TODO: Or we always cast (so pointer is not needed) or we find the place to be casted once, but constructor is not the place.
    sensor_laser_ptr_ = (SensorLaser2D*)(upperNodePtr());

	//check CaptureBase pointer is the appropriate one for this Processor
//	assert( capture_laser_ptr_ = dynamic_cast<CaptureLaser2D*>(capture_ptr_) && "Invalid Capture Type pointer" );
    capture_laser_ptr_ = (CaptureLaser2D*)(_capture_ptr);
	
    std::cout << "Extracting laser features..." << std::endl;

    //variables
    std::list <laserscanutils::Corner> corners;

    //extract corners from range data
    this->extractCorners(corners);
    //std::cout << corners.size() << " corners extracted" << std::endl;

    //generate a feature for each corner
    this->createFeatures(corners);
    //std::cout << getFeatureListPtr()->size() << " Features created" << std::endl;

}

void ProcessorLaser2D::establishConstraints(CaptureBase* _capture_ptr_)
{
	//check CaptureBase pointer is the appropriate one for this Processor
//	assert( capture_laser_ptr_ = dynamic_cast<CaptureLaser2D*>(capture_ptr_) && "Invalid Capture Type pointer" );
    capture_laser_ptr_ = (CaptureLaser2D*)(_capture_ptr_);
	
    std::cout << "Establishing constraints to laser features..." << std::endl;

    //Establish constraints for each feature
    //establishConstraints();
    this->establishConstraintsMHTree();
    //std::cout << "Constraints created" << std::endl;

}

unsigned int ProcessorLaser2D::extractCorners(std::list<laserscanutils::Corner> & _corner_list) const
{
//     std::list < laserscanutils::Line > line_list;
//
//     laserscanutils::extractLines(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), ranges_, line_list);
//     return laserscanutils::extractCorners(laser_ptr_->getScanParams(), laser_ptr_->getCornerAlgParams(), line_list, _corner_list);
    return laserscanutils::extractCorners(sensor_laser_ptr_->getScanParams(), sensor_laser_ptr_->getCornerAlgParams(), capture_laser_ptr_->getRanges(), _corner_list);
}

unsigned int ProcessorLaser2D::extractLines(std::list<laserscanutils::Line> & _line_list) const
{
    return laserscanutils::extractLines(sensor_laser_ptr_->getScanParams(), sensor_laser_ptr_->getCornerAlgParams().line_params_, capture_laser_ptr_->getRanges(), _line_list);
}

void ProcessorLaser2D::createFeatures(std::list<laserscanutils::Corner> & _corner_list) const
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
        Scalar L1 = corner_it->line_1_.length();
        Scalar L2 = corner_it->line_2_.length();
        Scalar cov_angle_line1 = 12 * corner_it->line_1_.error_ / (pow(L1,2) * (pow(corner_it->line_1_.np_,3)-pow(corner_it->line_1_.np_,2)));
        Scalar cov_angle_line2 = 12 * corner_it->line_2_.error_ / (pow(L2,2) * (pow(corner_it->line_2_.np_,3)-pow(corner_it->line_2_.np_,2)));


        //init cov in corner coordinates
        cov_mat << corner_it->line_1_.error_ + cov_angle_line1 * L1 * L1 / 4, 0, 0, 0,
                   0, corner_it->line_2_.error_ + cov_angle_line2 * L2 * L2 / 4, 0, 0,
                   0, 0, cov_angle_line1 + cov_angle_line2, 0,
                   0, 0, 0, cov_angle_line1 + cov_angle_line2;

        //std::cout << "New feature: " << meas.transpose() << std::endl;
        //std::cout << cov_mat << std::endl;

        // Rotate covariance
        R.topLeftCorner<2,2>() = Eigen::Rotation2D<Scalar>(corner_it->orientation_).matrix();
        cov_mat.topLeftCorner<3,3>() = R.transpose() * cov_mat.topLeftCorner<3,3>() * R;

        //std::cout << "rotated covariance: " << std::endl;
        //std::cout << cov_mat << std::endl;

        //TODO: add the rest of descriptors and struct atributes
        capture_laser_ptr_->addFeature((FeatureBase*) (new FeatureCorner2D(meas, 10*cov_mat)));
    }
}

void ProcessorLaser2D::establishConstraintsMHTree()
{
    if (capture_laser_ptr_->getFeatureListPtr()->size() != 0)
    {
        //local declarations
        Scalar prob, dm2;
        unsigned int ii, jj;
        std::map<unsigned int, unsigned int> ft_lk_pairs;
        std::vector<bool> associated_mask;
        Scalar sq24 = sqrt(2) / 4;
        Eigen::MatrixXs corners(3,4);
                    // Center seen from corners (see LandmarkContainer.h)
                    // A                                         // B                                         // C                                         // D
        corners <<  sq24 * (CONTAINER_LENGTH + CONTAINER_WIDTH), sq24 * (CONTAINER_LENGTH + CONTAINER_WIDTH), sq24 * (CONTAINER_LENGTH + CONTAINER_WIDTH), sq24 * (CONTAINER_LENGTH + CONTAINER_WIDTH),
                   -sq24 * (CONTAINER_LENGTH - CONTAINER_WIDTH), sq24 * (CONTAINER_LENGTH - CONTAINER_WIDTH),-sq24 * (CONTAINER_LENGTH - CONTAINER_WIDTH), sq24 * (CONTAINER_LENGTH - CONTAINER_WIDTH),
                   -M_PI / 4,                                   -3 * M_PI / 4,                                3 * M_PI / 4,                                M_PI / 4;
//        corners << -CONTAINER_LENGTH / 2, CONTAINER_LENGTH / 2, CONTAINER_LENGTH / 2,-CONTAINER_LENGTH / 2,
//                   -CONTAINER_WIDTH / 2, -CONTAINER_WIDTH / 2,  CONTAINER_WIDTH / 2,  CONTAINER_WIDTH / 2,
//                    M_PI / 4,             3 * M_PI / 4,        -3 * M_PI / 4,        -M_PI / 4;
        Eigen::MatrixXs rotated_corners = corners;
        Eigen::VectorXs squared_mahalanobis_distances;

        // COMPUTING ALL EXPECTED FEATURES
        std::map<LandmarkBase*, Eigen::Vector4s> expected_features;
        std::map<LandmarkBase*, Eigen::Matrix3s> expected_features_covs;
        unsigned int i = 0;
        for (auto j_it = getProblem()->getMapPtr()->getLandmarkListPtr()->begin(); j_it != getProblem()->getMapPtr()->getLandmarkListPtr()->end(); j_it++, i++)
            computeExpectedFeature(*j_it, expected_features[*j_it], expected_features_covs[*j_it]);

        // SETTING ASSOCIATION TREE
        std::map<unsigned int, FeatureCorner2D*> features_map;
        std::map<unsigned int, LandmarkBase*> landmarks_map;
        std::map<unsigned int, unsigned int> landmarks_index_map;

        //tree object allocation and sizing
        AssociationTree tree;
        tree.resize( capture_laser_ptr_->getFeatureListPtr()->size() , getProblem()->getMapPtr()->getLandmarkListPtr()->size() );

        //set independent probabilities between feature-landmark pairs
        ii=0;
        for (auto feature_it = capture_laser_ptr_->getFeatureListPtr()->begin(); feature_it != capture_laser_ptr_->getFeatureListPtr()->end(); feature_it++, ii++) //ii runs over extracted feature
        {
            features_map[ii] = (FeatureCorner2D*)(*feature_it);
            //std::cout << "Feature: " << (*i_it)->nodeId() << std::endl << (*i_it)->getMeasurement().head(3).transpose() << std::endl;
            jj = 0;
            for (auto landmark_it = getProblem()->getMapPtr()->getLandmarkListPtr()->begin(); landmark_it != getProblem()->getMapPtr()->getLandmarkListPtr()->end(); landmark_it++, jj++)
            {
                if ((*landmark_it)->getType() == LANDMARK_CORNER)
                {
                    landmarks_map[jj] = (*landmark_it);
                    landmarks_index_map[jj] = 0;
                    //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << std::endl;
                    //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                    if (fabs(pi2pi(((FeatureCorner2D*)(*feature_it))->getAperture() - (*landmark_it)->getDescriptor(0))) < MAX_ACCEPTED_APERTURE_DIFF)
                    {
                        dm2 = computeSquaredMahalanobisDistances(*feature_it, expected_features[*landmark_it], expected_features_covs[*landmark_it], Eigen::MatrixXs::Zero(3,1))(0);//Mahalanobis squared
                        prob = (dm2 < 5*5 ? 5*erfc( sqrt(dm2/2) ) : 0); //prob = erfc( sqrt(dm2/2) ); //prob = erfc( sqrt(dm2)/1.4142136 );// sqrt(2) = 1.4142136
                        tree.setScore(ii,jj,prob);
                    }
                    else
                        tree.setScore(ii,jj,0.);//prob to 0
                }
                else if ((*landmark_it)->getType() == LANDMARK_CONTAINER)
                {
                    //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << " " << jj+1 << " " << jj+2 << " " << jj+3 << std::endl;
                    // Resize tree (expected 1 target for each landmark but containers have 4 targets)
                    if (feature_it == capture_laser_ptr_->getFeatureListPtr()->begin())
                        tree.resize( tree.numDetections() , tree.numTargets()+3 );

                    //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                    if (fabs(pi2pi(((FeatureCorner2D*)(*feature_it))->getAperture() - 3 * M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF)
                    {
                        // Rotate corners (seen from feature coordinates)
                        //rotated_corners.topRows(2) = Eigen::Rotation2D<Scalar>(expected_features[*landmark_it](2)).matrix() * corners.topRows(2);
                        rotated_corners = corners;
                        squared_mahalanobis_distances = computeSquaredMahalanobisDistances((*feature_it), expected_features[*landmark_it], expected_features_covs[*landmark_it], rotated_corners);

                        for (unsigned int c = 0; c < 4; c++, jj++)
                        {
                            landmarks_map[jj] = (*landmark_it);
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
                            landmarks_map[jj] = (*landmark_it);
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
        for (auto i_it = capture_laser_ptr_->getFeatureListPtr()->begin(); i_it != capture_laser_ptr_->getFeatureListPtr()->end(); i_it++, ii++) //ii runs over extracted feature
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
                    associed_feature->addConstraint(new ConstraintCorner2D(associed_feature,                                // feature pointer
                                                                           (LandmarkCorner2D*)(associed_landmark)));    // landmark pointer

                else if (associed_landmark->getType() == LANDMARK_CONTAINER)
                    associed_feature->addConstraint(new ConstraintContainer(associed_feature,                       //feature pointer
                                                                                (LandmarkContainer*)(associed_landmark), //landmark pointer
                                                                                associed_landmark_index ));                 // corner index
            }
        }

        // CREATING NEW LANDMARKS
        Eigen::Vector3s feature_global_pose;

        // Global transformation
        Eigen::Vector2s t_robot = capture_laser_ptr_->getFramePtr()->getPPtr()->getVector().head(2);
        Eigen::Matrix2s R_robot = Eigen::Rotation2D<Scalar>(*(capture_laser_ptr_->getFramePtr()->getOPtr()->getPtr())).matrix();
        Scalar robot_orientation = *(capture_laser_ptr_->getFramePtr()->getOPtr()->getPtr());

        // Sensor transformation
        Eigen::Vector2s t_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
        Eigen::Rotation2D<Scalar> S_R( getSensorPtr()->getOPtr()->getVector()(0) );
        Eigen::Matrix2s R_sensor = S_R.matrix();
        Scalar sensor_orientation = *(capture_laser_ptr_->getSensorPtr()->getOPtr()->getPtr());

        // NEW CORNERS
        for (auto it=new_corner_landmarks.begin(); it!=new_corner_landmarks.end();it++)
        {
            //get feature on sensor frame
            const Eigen::Vector2s& feature_position = (*it)->getMeasurement().head(2);
            const Scalar& feature_orientation = (*it)->getMeasurement()(2);

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
            const Scalar& feature_orientation = new_container_landmarks.at(i).first->getMeasurement()(2);

            //translate feature position and orientation to world (global)
            feature_global_pose.head(2) = R_robot * (R_sensor * feature_position + t_sensor) + t_robot;
            feature_global_pose(2) = pi2pi(feature_orientation + robot_orientation + sensor_orientation);

            createContainerLandmark(new_container_landmarks.at(i).first, feature_global_pose, (LandmarkCorner2D*)(new_container_landmarks.at(i).second), new_container_indexs.at(i).first, new_container_indexs.at(i).second);
        }
    }
}

//Eigen::VectorXs ProcessorLaser2D::computePrior(const TimeStamp& _now) const
//{
//    return Eigen::Vector3s(1, 2, 3);
//}

void ProcessorLaser2D::computeExpectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_, Eigen::Matrix3s& expected_feature_cov_)
{
    FrameBase* frame_ptr = capture_laser_ptr_->getFramePtr();

    Eigen::Vector2s p_robot = frame_ptr->getPPtr()->getVector();
    Scalar      o_robot = *(frame_ptr->getOPtr()->getPtr());

    Eigen::Vector2s p_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
    Scalar      o_sensor = *(getSensorPtr()->getOPtr()->getPtr());
    Eigen::Rotation2D<Scalar> S_R( getSensorPtr()->getOPtr()->getVector()(0) );
    Eigen::Matrix2s R_sensor = S_R.matrix();

    Eigen::Vector2s p_landmark = _landmark_ptr->getPPtr()->getVector();
    Scalar      o_landmark = *(_landmark_ptr->getOPtr()->getPtr());

    Scalar o_rs = o_robot + o_sensor; // Sum of theta_sensor and theta_robot
    Eigen::Matrix2s R_sr = Eigen::Rotation2D<Scalar>(o_rs).matrix(); // Sum of theta_sensor and theta_robot

    Eigen::Vector2s p_robot_landmark = p_robot - p_landmark;

    // ------------------------ Expected feature
    expected_feature_.head(2) = - R_sr.transpose() * p_robot_landmark - R_sensor.transpose() * p_sensor;
    expected_feature_(2) = pi2pi(o_landmark - o_robot - o_sensor);

    if (_landmark_ptr->getType() == LANDMARK_CONTAINER)
    {
        expected_feature_(3) = 3 * M_PI / 2;
    }
    else if (_landmark_ptr->getType() == LANDMARK_CORNER)
    {
        expected_feature_(3) = ((LandmarkCorner2D*)(_landmark_ptr))->getAperture();
    }

    // ------------------------ Sigma
    // JACOBIAN
    Eigen::Matrix<Scalar, 3, 6> Jlr = Eigen::Matrix<Scalar, 3, 6>::Zero();
    Jlr.block<2,2>(0,0) = -R_sr.transpose();
    Jlr(2,2) = -1;
    Jlr.block<2,2>(0,3) = R_sr.transpose();
    Jlr(0,2) = -p_robot_landmark(0)*sin(o_rs) + p_robot_landmark(1)*cos(o_rs);
    Jlr(1,2) = -p_robot_landmark(0)*cos(o_rs) - p_robot_landmark(1)*sin(o_rs);
    Jlr(2,2) = 1;

    // SIGMA (filling upper diagonal only)
    Eigen::MatrixXs Sigma = Eigen::MatrixXs::Zero(6,6);
    // Sigma_ll
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0,0);
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma, 0,2);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma, 2,2);
    // Sigma_lr
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 0,3);
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 0,5);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getPPtr(), Sigma, 2,3);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 2,5);
    // Sigma_rr
    getProblem()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 3,3);
    getProblem()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 3,5);
    getProblem()->getCovarianceBlock(frame_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 5,5);

    expected_feature_cov_ = Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose();
}

Eigen::VectorXs ProcessorLaser2D::computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const LandmarkBase* _landmark_ptr, const Eigen::MatrixXs& _mu)
{
    assert(_mu.rows() == 3 && "mahalanobis distance with bad number of mu components");

    FrameBase* frame_ptr = capture_laser_ptr_->getFramePtr();

    Eigen::Vector2s p_robot = frame_ptr->getPPtr()->getVector();
    Scalar      o_robot = *(frame_ptr->getOPtr()->getPtr());

    Eigen::Vector2s p_sensor = getSensorPtr()->getPPtr()->getVector().head(2);
    Scalar      o_sensor = *(getSensorPtr()->getOPtr()->getPtr());
//    Eigen::Matrix2s R_sensor = getSensorPtr()->getRotationMatrix2D();
    Eigen::Rotation2D<Scalar> S_R( getSensorPtr()->getOPtr()->getVector()(0) );
    Eigen::Matrix2s R_sensor = S_R.matrix();

    Eigen::Vector2s p_landmark = _landmark_ptr->getPPtr()->getVector();
    Scalar      o_landmark = *(_landmark_ptr->getOPtr()->getPtr());

    const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
    const Scalar&      o_feature = _feature_ptr->getMeasurement()(2);

    Scalar o_rs = o_robot + o_sensor; // Sum of theta_sensor and theta_robot
    Eigen::Matrix2s R_sr = Eigen::Rotation2D<Scalar>(o_rs).matrix(); // Sum of theta_sensor and theta_robot
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
    Eigen::Matrix<Scalar, 3, 6> Jlr = Eigen::Matrix<Scalar, 3, 6>::Zero();
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
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0,0);
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma, 0,2);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma, 2,2);
    // Sigma_lr
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 0,3);
    getProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 0,5);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getPPtr(), Sigma, 2,3);
    getProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 2,5);
    // Sigma_rr
    getProblem()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getPPtr(), Sigma, 3,3);
    getProblem()->getCovarianceBlock(frame_ptr->getPPtr(), frame_ptr->getOPtr(), Sigma, 3,5);
    getProblem()->getCovarianceBlock(frame_ptr->getOPtr(), frame_ptr->getOPtr(), Sigma, 5,5);

    // MAHALANOBIS DISTANCES
    Eigen::Matrix3s iSigma_d = (Sigma_feature + Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose()).inverse();

    Eigen::VectorXs squared_mahalanobis_distances(_mu.cols());

    for (unsigned int i = 0; i < _mu.cols(); i++)
        squared_mahalanobis_distances(i) = (d - _mu.col(i)).transpose() * iSigma_d * (d - _mu.col(i));

    return squared_mahalanobis_distances;
}

Eigen::VectorXs ProcessorLaser2D::computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr, const Eigen::Vector4s& _expected_feature, const Eigen::Matrix3s& _expected_feature_cov, const Eigen::MatrixXs& _mu)
{
    assert(_mu.rows() == 3 && "mahalanobis distance with bad number of mu components");

    const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
    const Scalar&      o_feature = _feature_ptr->getMeasurement()(2);

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

bool ProcessorLaser2D::fitNewContainer(FeatureCorner2D* _corner_feature_ptr, LandmarkCorner2D*& _corner_landmark_ptr, int& feature_corner_idx, int& landmark_corner_idx)
{
    //std::cout << "Trying container... aperture = " << _corner_ptr->getMeasurement()(3) << std::endl;

    // It has to be 90º corner feature
    if (std::fabs(pi2pi(_corner_feature_ptr->getMeasurement()(3) + M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF)
    {
        // Check all existing corners searching a container
        for (auto landmark_it = getProblem()->getMapPtr()->getLandmarkListPtr()->rbegin(); landmark_it != getProblem()->getMapPtr()->getLandmarkListPtr()->rend(); landmark_it++)
        {
            if ((*landmark_it)->getType() == LANDMARK_CORNER // should be a corner
                    && std::fabs(pi2pi(((LandmarkCorner2D*)(*landmark_it))->getAperture() + M_PI / 2)) < MAX_ACCEPTED_APERTURE_DIFF) // should be a corner
            {
                //std::cout << "landmark " << (*landmark_it)->nodeId() << std::endl;
                Scalar SQ2 = sqrt(2)/2;
                Eigen::MatrixXs corners_relative_positions(3,6);
                                              // FROM A (see LandmarkContainer.h)                                                        // FROM B
                                              // Large side           // Short side          // Diagonal                                 // Large side           // Short side          // Diagonal
                corners_relative_positions << SQ2 * CONTAINER_LENGTH, SQ2 * CONTAINER_WIDTH, SQ2 * (CONTAINER_LENGTH + CONTAINER_WIDTH), SQ2 * CONTAINER_LENGTH, SQ2 * CONTAINER_WIDTH, SQ2 * (CONTAINER_LENGTH + CONTAINER_WIDTH),
                                             -SQ2 * CONTAINER_LENGTH, SQ2 * CONTAINER_WIDTH, SQ2 * (CONTAINER_WIDTH - CONTAINER_LENGTH), SQ2 * CONTAINER_LENGTH,-SQ2 * CONTAINER_WIDTH, SQ2 * (CONTAINER_LENGTH - CONTAINER_WIDTH),
                                              M_PI / 2,              -M_PI / 2,              M_PI,                                      -M_PI / 2,               M_PI / 2,              M_PI;

                Eigen::Matrix2s R_feature = Eigen::Rotation2D<Scalar>(_corner_feature_ptr->getMeasurement()(2)).matrix();
                corners_relative_positions = - corners_relative_positions;
                corners_relative_positions.topRows(2) = R_feature * corners_relative_positions.topRows(2);

                Eigen::VectorXs squared_mahalanobis_distances = computeSquaredMahalanobisDistances(_corner_feature_ptr, (*landmark_it), corners_relative_positions);

//                std::cout << "squared_mahalanobis_distances " << std::endl << squared_mahalanobis_distances << std::endl;
//                std::cout << "probabilities " << std::endl;
//                for (unsigned int i = 0; i < 6; i++)
//                    std::cout << erfc( sqrt(squared_mahalanobis_distances(i)/2) ) << std::endl;

                Scalar SMD_threshold = 8;
                if (squared_mahalanobis_distances(0) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(0)/2) ) > 0.8 )
                {
                    std::cout << "   large side! prob =  " << erfc( sqrt(squared_mahalanobis_distances(0)/2.0)) << std::endl;
                    feature_corner_idx = 0;
                    landmark_corner_idx = 1;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(1) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(1)/2) ) > 0.8 )
                {
                    std::cout << "   short side!  prob = " << erfc( sqrt(squared_mahalanobis_distances(1)/2)) << std::endl;
                    feature_corner_idx = 2;
                    landmark_corner_idx = 1;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(2) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(2)/2) ) > 0.8 )
                {
                    std::cout << "   diagonal!  prob = " << erfc( sqrt(squared_mahalanobis_distances(2)/2)) << std::endl;
                    feature_corner_idx = 0;
                    landmark_corner_idx = 2;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(3) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(3)/2) ) > 0.8 )
                {
                    std::cout << "   large side! prob = " << erfc( sqrt(squared_mahalanobis_distances(3)/2)) << std::endl;
                    feature_corner_idx = 1;
                    landmark_corner_idx = 0;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(4) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(4)/2) ) > 0.8 )
                {
                    std::cout << "   short side!  prob = " << erfc( sqrt(squared_mahalanobis_distances(4)/2)) << std::endl;
                    feature_corner_idx = 1;
                    landmark_corner_idx = 2;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
                if (squared_mahalanobis_distances(5) < SMD_threshold ) //erfc( sqrt(squared_mahalanobis_distances(5)/2) ) > 0.8 )
                {
                    std::cout << "   diagonal!  prob = " << erfc( sqrt(squared_mahalanobis_distances(5)/2)) << std::endl;
                    feature_corner_idx = 1;
                    landmark_corner_idx = 3;
                    _corner_landmark_ptr = (LandmarkCorner2D*)(*landmark_it);
                    return true;
                }
            }
        }
    }
    return false;
}

void ProcessorLaser2D::createCornerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose)
{
    //Create new landmark
    LandmarkCorner2D* new_landmark = new LandmarkCorner2D(new StateBlock(_feature_global_pose.head(2)),
                                                          new StateBlock(_feature_global_pose.tail(1)),
                                                          _corner_ptr->getMeasurement()(3));
    //Constraint with the new landmark
    _corner_ptr->addConstraint(new ConstraintCorner2D(_corner_ptr, new_landmark));
    //Add it to the map
    getProblem()->getMapPtr()->addLandmark((LandmarkBase*)new_landmark);

    // Initialize landmark covariance // TODO: has it sense???
    Eigen::MatrixXs Sigma_robot = Eigen::MatrixXs::Zero(3,3);
    getProblem()->getCovarianceBlock(capture_laser_ptr_->getFramePtr()->getPPtr(), capture_laser_ptr_->getFramePtr()->getPPtr(), Sigma_robot, 0,0);
    getProblem()->getCovarianceBlock(capture_laser_ptr_->getFramePtr()->getPPtr(), capture_laser_ptr_->getFramePtr()->getOPtr(), Sigma_robot, 0,2);
    getProblem()->getCovarianceBlock(capture_laser_ptr_->getFramePtr()->getOPtr(), capture_laser_ptr_->getFramePtr()->getOPtr(), Sigma_robot, 2,2);
    Sigma_robot.block<1,2>(2,0) = Sigma_robot.block<2,1>(0,2).transpose();

    Eigen::Matrix3s R_robot3D = Eigen::Matrix3s::Identity();
    R_robot3D.block<2,2>(0,0) = Eigen::Rotation2D<Scalar>(*(capture_laser_ptr_->getFramePtr()->getOPtr()->getPtr())).matrix();
    Eigen::Matrix3s Sigma_landmark = Sigma_robot + R_robot3D.transpose() * _corner_ptr->getMeasurementCovariance().topLeftCorner<3,3>() * R_robot3D;

    getProblem()->addCovarianceBlock(new_landmark->getPPtr(), new_landmark->getPPtr(), Sigma_landmark.topLeftCorner<2,2>());
    getProblem()->addCovarianceBlock(new_landmark->getPPtr(), new_landmark->getOPtr(), Sigma_landmark.block<2,1>(0,2));
    getProblem()->addCovarianceBlock(new_landmark->getOPtr(), new_landmark->getOPtr(), Sigma_landmark.block<1,1>(2,2));
}

void ProcessorLaser2D::createContainerLandmark(FeatureCorner2D* _corner_ptr, const Eigen::Vector3s& _feature_global_pose, LandmarkCorner2D* _old_corner_landmark_ptr, int& _feature_idx, int& _corner_idx)
{
    assert(_old_corner_landmark_ptr != nullptr && "fitting result = true but corner found is nullptr");

    // create new landmark
    Eigen::Vector3s corner_pose;
    corner_pose.head(2) = _old_corner_landmark_ptr->getPPtr()->getVector();
    corner_pose(2) = *(_old_corner_landmark_ptr->getOPtr()->getPtr());

    LandmarkContainer* new_landmark = new LandmarkContainer(new StateBlock(Eigen::Vector2s::Zero()),
                                                            new StateBlock(Eigen::Vector1s::Zero()),
                                                            _feature_global_pose,
                                                            corner_pose,
                                                            _feature_idx,
                                                            _corner_idx,
                                                            CONTAINER_WIDTH,
                                                            CONTAINER_LENGTH);
    // create new constraint (feature to container)
    _corner_ptr->addConstraint(new ConstraintContainer(_corner_ptr, new_landmark,_feature_idx));

    // add new landmark in the map
    getProblem()->getMapPtr()->addLandmark((LandmarkBase*)new_landmark);

    // initialize container covariance with landmark corner covariance // TODO: has it sense???
    Eigen::MatrixXs Sigma_landmark = Eigen::MatrixXs::Zero(3,3);
    getProblem()->getCovarianceBlock(_old_corner_landmark_ptr->getPPtr(), _old_corner_landmark_ptr->getPPtr(), Sigma_landmark, 0,0);
    getProblem()->getCovarianceBlock(_old_corner_landmark_ptr->getPPtr(), _old_corner_landmark_ptr->getOPtr(), Sigma_landmark, 0,2);
    getProblem()->getCovarianceBlock(_old_corner_landmark_ptr->getOPtr(), _old_corner_landmark_ptr->getOPtr(), Sigma_landmark, 2,2);
    Sigma_landmark.block<1,2>(2,0) = Sigma_landmark.block<2,1>(0,2).transpose();

    getProblem()->addCovarianceBlock(new_landmark->getPPtr(), new_landmark->getPPtr(), Sigma_landmark.topLeftCorner<2,2>());
    getProblem()->addCovarianceBlock(new_landmark->getPPtr(), new_landmark->getOPtr(), Sigma_landmark.block<2,1>(0,2));
    getProblem()->addCovarianceBlock(new_landmark->getOPtr(), new_landmark->getOPtr(), Sigma_landmark.block<1,1>(2,2));


    // ERASING LANDMARK CORNER
    // change all constraints from corner landmark by new corner container
    for (auto ctr_it = _old_corner_landmark_ptr->getConstrainedByListPtr()->begin(); ctr_it != _old_corner_landmark_ptr->getConstrainedByListPtr()->end(); ctr_it++)
    {
        // create new constraint to landmark container
        (*ctr_it)->getFeaturePtr()->addConstraint(new ConstraintContainer((*ctr_it)->getFeaturePtr(), new_landmark, _corner_idx));
    }
    // Remove corner landmark (it will remove all old constraints)
    getProblem()->getMapPtr()->removeLandmark(_old_corner_landmark_ptr);
}


ProcessorBase* ProcessorLaser2D::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorParamsLaser2D* params_ptr = (ProcessorParamsLaser2D*)_params;
    ProcessorLaser2D* prc_ptr = new ProcessorLaser2D();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}


} // namespace wolf
