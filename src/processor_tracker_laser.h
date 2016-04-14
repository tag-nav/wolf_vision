/*
 * processor_laser_corners.h
 *
 *  Created on: Apr 4, 2016
 *      Author: jvallve
 */

#ifndef SRC_PROCESSOR_TRACKER_LASER_H_
#define SRC_PROCESSOR_TRACKER_LASER_H_

// Wolf includes
#include "sensor_laser_2D.h"
#include "capture_laser_2D.h"
#include "feature_corner_2D.h"
#include "landmark_corner_2D.h"
#include "constraint_corner_2D.h"
#include "state_block.h"
#include "data_association/association_tree.h"

//laser_scan_utils
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/corner_detector.h"
#include "processor_tracker_landmark.h"

namespace wolf
{

//some consts.. TODO: this tuning params should be grouped in a struct and passed to the class from ros node, at constructor level
const WolfScalar aperture_error_th_ = 20.0 * M_PI / 180.; //20 degrees
const WolfScalar angular_error_th_ = 10.0 * M_PI / 180.; //10 degrees;
const WolfScalar position_error_th_ = 1;
const WolfScalar min_features_ratio_th_ = 0.5;

class ProcessorTrackerLaser : public ProcessorTrackerLandmark
{
    private:
        laserscanutils::ScanParams scan_params_;
        laserscanutils::ExtractCornerParams corner_alg_params_;

        Eigen::Matrix3s R_sensor_world_, R_world_sensor_;
        Eigen::Matrix3s R_robot_sensor_;
        Eigen::Vector3s t_sensor_world_, t_world_sensor_;
        Eigen::Vector3s t_robot_sensor_;
        Eigen::Vector3s t_world_robot_;
        bool extrinsics_transformation_computed_;

    public:
        ProcessorTrackerLaser(const laserscanutils::ScanParams& _scan_params,
                              const laserscanutils::ExtractCornerParams& _corner_alg_params) :
                ProcessorTrackerLandmark(PRC_TRACKER_LIDAR, 0), scan_params_(_scan_params), corner_alg_params_(
                        _corner_alg_params), extrinsics_transformation_computed_(false)
        {
        }

        virtual ~ProcessorTrackerLaser()
        {
        }

    protected:

        virtual void preProcess()
        {
            // extract corners of incoming
            extractCorners((CaptureLaser2D*)incoming_ptr_, new_features_incoming_);

            // get the pose of the vehicle in incoming timestamp

            // compute transformations
            t_world_robot_ = getWolfProblem()->getStateAtTimeStamp(incoming_ptr_->getTimeStamp());

            // world_robot
            Eigen::Matrix3s R_world_robot = Eigen::Matrix3s::Identity();
            R_world_robot.topLeftCorner(2, 2) = Eigen::Rotation2Ds(t_world_robot_(2)).matrix();

            // robot_sensor (to be computed once if extrinsics are fixed and not dynamic)
            if (getSensorPtr()->isExtrinsicDynamic() || !getSensorPtr()->getPPtr()->isFixed()
                    || !getSensorPtr()->getOPtr()->isFixed() || !extrinsics_transformation_computed_)
            {
                t_robot_sensor_.head(2) = getSensorPtr()->getPPtr()->getVector();
                t_robot_sensor_(2) = getSensorPtr()->getOPtr()->getVector()(0);
                R_robot_sensor_ = Eigen::Matrix3s::Identity();
                R_robot_sensor_.topLeftCorner(2, 2) = Eigen::Rotation2Ds(t_robot_sensor_(2)).matrix();

                extrinsics_transformation_computed_ = true;
            }
            // global_sensor
            R_world_sensor_ = R_world_robot * R_robot_sensor_;
            t_world_sensor_ = t_world_robot_ + R_robot_sensor_ * t_robot_sensor_;

            // sensor_global
            R_sensor_world_ = R_robot_sensor_.transpose() * R_world_robot.transpose();
            t_sensor_world_ = -R_sensor_world_ * t_world_robot_ - R_robot_sensor_.transpose() * t_robot_sensor_;

        }
        virtual void postProcess()
        {

        }

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences)
        {
            if (!new_features_incoming_.empty())
            {
                //local declarations
                WolfScalar prob, dm2;
                unsigned int ii, jj;
                std::map<unsigned int, unsigned int> ft_lk_pairs;
                std::vector<bool> associated_mask;
                Eigen::VectorXs squared_mahalanobis_distances;

                // COMPUTING ALL EXPECTED FEATURES
                std::map<LandmarkBase*, Eigen::Vector4s> expected_features;
                std::map<LandmarkBase*, Eigen::Matrix3s> expected_features_covs;
                for (auto landmark : _landmark_list_in)
                    expectedFeature(landmark, expected_features[landmark], expected_features_covs[landmark]);

                // SETTING ASSOCIATION TREE
                std::map<unsigned int, FeatureBaseIter> features_map;
                std::map<unsigned int, LandmarkBaseIter> landmarks_map;
                std::map<unsigned int, unsigned int> landmarks_index_map;

                //tree object allocation and sizing
                AssociationTree tree;
                tree.resize(new_features_incoming_.size(), _landmark_list_in.size());

                //set independent probabilities between feature-landmark pairs
                ii = 0;
                for (auto feature_it = new_features_incoming_.begin(); feature_it != new_features_incoming_.end();
                        feature_it++, ii++) //ii runs over extracted feature
                {
                    features_map[ii] = feature_it;
                    //std::cout << "Feature: " << (*i_it)->nodeId() << std::endl << (*i_it)->getMeasurement().head(3).transpose() << std::endl;
                    jj = 0;
                    for (auto landmark_it = _landmark_list_in.begin(); landmark_it != _landmark_list_in.end();
                            landmark_it++, jj++)
                    {
                        if ((*landmark_it)->getType() == LANDMARK_CORNER)
                        {
                            landmarks_map[jj] = landmark_it;
                            landmarks_index_map[jj] = 0;
                            //std::cout << "Landmark: " << (*j_it)->nodeId() << " - jj: " << jj << std::endl;
                            //If aperture difference is small enough, proceed with Mahalanobis distance. Otherwise Set prob to 0 to force unassociation
                            if (fabs(
                                    pi2pi(((FeatureCorner2D*)(*feature_it))->getAperture()
                                            - (*landmark_it)->getDescriptor(0))) < aperture_error_th_)
                            {
                                dm2 = computeSquaredMahalanobisDistances(*feature_it, expected_features[*landmark_it],
                                                                         expected_features_covs[*landmark_it],
                                                                         Eigen::MatrixXs::Zero(3, 1))(0); //Mahalanobis squared
                                prob = (dm2 < 5 * 5 ? 5 * erfc(sqrt(dm2 / 2)) : 0); //prob = erfc( sqrt(dm2/2) ); //prob = erfc( sqrt(dm2)/1.4142136 );// sqrt(2) = 1.4142136
                                tree.setScore(ii, jj, prob);
                            }
                            else
                                tree.setScore(ii, jj, 0.); //prob to 0
                        }
                    }
                }

                // Grows tree and make association pairs
                std::cout << "solving tree" << std::endl;
                tree.solve(ft_lk_pairs, associated_mask);

                //print tree & score table
                std::cout << "------------- TREE SOLVED ---------" << std::endl;
                std::cout << new_features_incoming_.size() << " new corners:" << std::endl;
                for (auto new_feature : new_features_incoming_)
                    std::cout << "\tnew feature " << new_feature->nodeId() << std::endl;
                std::cout << ft_lk_pairs.size() << " pairs:"<< std::endl;
                for (auto pair : ft_lk_pairs)
                    std::cout << "\tfeature " << pair.first << " & landmark " << pair.second << std::endl;

                //tree.printTree();
                //tree.printScoreTable();

                // Vector of new landmarks to be created
                std::vector<FeatureCorner2D*> new_corner_landmarks(0);

                // ESTABLISH CORRESPONDENCES
                for (auto pair : ft_lk_pairs)
                {
                    // match
                    matches_landmark_from_incoming_[*features_map[pair.first]] = LandmarkMatch(*landmarks_map[pair.second], tree.getScore(pair.first, pair.second));

                    // move matched feature to list
                    _feature_list_out.splice(_feature_list_out.end(), new_features_incoming_, features_map[pair.first]);
                }

                std::cout << new_features_incoming_.size() << " new corners:" << std::endl;
                for (auto new_feature : new_features_incoming_)
                    std::cout << "\tnew feature " << new_feature->nodeId() << std::endl;
            }
            return matches_landmark_from_incoming_.size();
        }

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _capture_ptr Capture for feature detection. Defaults to incoming_ptr_.
         * \param _new_features_list The list of detected Features. Defaults to member new_features_list_.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features)
        {
            // already computed since each scan is computed in preprocess(), new corners are classified in findLandmarks()
            return new_features_last_.size();
        }

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr)
        {
            // compute feature global pose
            Eigen::Vector3s feature_global_pose = R_world_sensor_ * _feature_ptr->getMeasurement() + t_world_sensor_;

            // Create new landmark
            return new LandmarkCorner2D(new StateBlock(feature_global_pose.head(2)),
                                        new StateBlock(feature_global_pose.tail(1)), _feature_ptr->getMeasurement()(3));
        }

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
        {
            return new ConstraintCorner2D(_feature_ptr, (LandmarkCorner2D*)(_landmark_ptr));
        }

    private:

        void extractCorners(const CaptureLaser2D* _capture_laser_ptr, FeatureBaseList& _corner_list);

        void expectedFeature(LandmarkBase* _landmark_ptr, Eigen::Vector4s& expected_feature_,
                             Eigen::Matrix3s& expected_feature_cov_)
        {
            // closer keyframe with computed covariance
            FrameBase* key_frame_ptr = origin_ptr_->getFramePtr();

            // ------------ expected feature measurement
            expected_feature_.head(3) = R_sensor_world_ * (_landmark_ptr->getPPtr()->getVector() - t_world_sensor_);
            expected_feature_(3) = ((LandmarkCorner2D*)(_landmark_ptr))->getAperture();

            // ------------ expected feature covariance
            // Sigma
            Eigen::MatrixXs Sigma = Eigen::MatrixXs::Zero(6, 6);
            // If all covariance blocks are stored wolfproblem (filling upper diagonal only)
            if (
            // Sigma_ll
            getWolfProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getPPtr(), Sigma, 0, 0)
                    && getWolfProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), _landmark_ptr->getOPtr(), Sigma,
                                                            0, 2)
                    && getWolfProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), _landmark_ptr->getOPtr(), Sigma,
                                                            2, 2)
                    &&
                    // Sigma_lr
                    getWolfProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), key_frame_ptr->getPPtr(), Sigma, 0,
                                                         3)
                    && getWolfProblem()->getCovarianceBlock(_landmark_ptr->getPPtr(), key_frame_ptr->getOPtr(), Sigma,
                                                            0, 5)
                    && getWolfProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), key_frame_ptr->getPPtr(), Sigma,
                                                            2, 3)
                    && getWolfProblem()->getCovarianceBlock(_landmark_ptr->getOPtr(), key_frame_ptr->getOPtr(), Sigma,
                                                            2, 5)
                    &&
                    // Sigma_rr
                    getWolfProblem()->getCovarianceBlock(key_frame_ptr->getPPtr(), key_frame_ptr->getPPtr(), Sigma, 3,
                                                         3)
                    && getWolfProblem()->getCovarianceBlock(key_frame_ptr->getPPtr(), key_frame_ptr->getOPtr(), Sigma,
                                                            3, 5)
                    && getWolfProblem()->getCovarianceBlock(key_frame_ptr->getOPtr(), key_frame_ptr->getOPtr(), Sigma,
                                                            5, 5))
            {

                // Jacobian
                Eigen::Vector2s p_robot_landmark = t_world_robot_.head(2) - _landmark_ptr->getPPtr()->getVector();
                Eigen::Matrix<WolfScalar, 3, 6> Jlr = Eigen::Matrix<WolfScalar, 3, 6>::Zero();
                Jlr.block<3, 3>(0, 3) = -R_world_sensor_.transpose();
                Jlr.block<3, 3>(0, 3) = R_world_sensor_.transpose();
                Jlr(0, 2) = -p_robot_landmark(0) * sin(t_world_sensor_(2))
                        + p_robot_landmark(1) * cos(t_world_sensor_(2));
                Jlr(1, 2) = -p_robot_landmark(0) * cos(t_world_sensor_(2))
                        - p_robot_landmark(1) * sin(t_world_sensor_(2));

                // measurement covariance
                expected_feature_cov_ = Jlr * Sigma.selfadjointView<Eigen::Upper>() * Jlr.transpose();
            }
            // Any covariance block is not stored in wolfproblem -> Identity()
            else
                expected_feature_cov_ = Eigen::Matrix3s::Identity();
        }

        Eigen::VectorXs computeSquaredMahalanobisDistances(const FeatureBase* _feature_ptr,
                                                           const Eigen::Vector4s& _expected_feature,
                                                           const Eigen::Matrix3s& _expected_feature_cov,
                                                           const Eigen::MatrixXs& _mu)
        {
            assert(_mu.rows() == 3 && "mahalanobis distance with bad number of mu components");

            const Eigen::Vector2s& p_feature = _feature_ptr->getMeasurement().head(2);
            const WolfScalar& o_feature = _feature_ptr->getMeasurement()(2);

            // ------------------------ d
            Eigen::Vector3s d;
            d.head(2) = p_feature - _expected_feature.head(2);
            d(2) = pi2pi(o_feature - _expected_feature(2));

            //    std::cout << "feature = " << p_feature.transpose() << o_feature << std::endl;
            //    std::cout << "d = " << d.transpose() << std::endl;

            // ------------------------ Sigma_d
            Eigen::Matrix3s iSigma_d = (_feature_ptr->getMeasurementCovariance().topLeftCorner<3, 3>()
                    + _expected_feature_cov).inverse();

            Eigen::VectorXs squared_mahalanobis_distances(_mu.cols());

            for (unsigned int i = 0; i < _mu.cols(); i++)
                squared_mahalanobis_distances(i) = (d - _mu.col(i)).transpose() * iSigma_d * (d - _mu.col(i));

            return squared_mahalanobis_distances;
        }
};

} // namespace wolf

#endif /* SRC_PROCESSOR_TRACKER_LASER_H_ */
