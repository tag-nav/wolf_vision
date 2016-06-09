#include "processor_tracker_landmark_polyline.h"

namespace wolf
{

void ProcessorTrackerLandmarkPolyline::preProcess()
{
    //std::cout << "PreProcess: " << std::endl;

    // extract corners of incoming
    extractPolylines((CaptureLaser2D*)((incoming_ptr_)), polylines_incoming_);

    // compute transformations
    computeTransformations(incoming_ptr_->getTimeStamp());

    //std::cout << "PreProcess: incoming new features: " << polylines_incoming_.size() << std::endl;
}

void ProcessorTrackerLandmarkPolyline::computeTransformations(const TimeStamp& _ts)
{
    Eigen::Vector3s vehicle_pose = getProblem()->getStateAtTimeStamp(_ts);
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
    t_sensor_world_ = -R_robot_sensor_.transpose() * t_robot_sensor_ -R_sensor_world_ * t_world_robot_ ;

    //std::cout << "t_world_robot_ " << t_world_robot_.transpose() << std::endl;
    //std::cout << "t_robot_sensor_ " << t_robot_sensor_.transpose() << std::endl;
    //std::cout << "R_robot_sensor_ " << std::endl << R_robot_sensor_ << std::endl;
    //std::cout << "t_world_sensor_ " << t_world_sensor_.transpose() << std::endl;
    //std::cout << "R_world_sensor_ " << std::endl << R_world_sensor_ << std::endl;
    //std::cout << "t_sensor_world_ " << t_sensor_world_.transpose() << std::endl;
    //std::cout << "R_sensor_world_ " << std::endl << R_sensor_world_ << std::endl;

}

unsigned int ProcessorTrackerLandmarkPolyline::findLandmarks(const LandmarkBaseList& _landmarks_searched,
                                                           FeatureBaseList& _features_found,
                                                           LandmarkMatchMap& _feature_landmark_correspondences)
{
    //std::cout << "ProcessorTrackerLandmarkPolyline::findLandmarks: " << _landmarks_searched.size() << " features: " << polylines_incoming_.size()  << std::endl;

    // COMPUTING ALL EXPECTED FEATURES
    LandmarkBaseList not_matched_landmarks;
    std::map<LandmarkBase*, Eigen::MatrixXs> expected_features;
    std::map<LandmarkBase*, Eigen::MatrixXs> expected_features_covs;
    for (auto landmark : _landmarks_searched)
        if (landmark->getType() == LANDMARK_POLYLINE_2D)
        {
            not_matched_landmarks.push_back(landmark);
            expected_features[landmark] = Eigen::MatrixXs(3, ((LandmarkPolyline2D*)landmark)->getNPoints());
            expected_features_covs[landmark] = Eigen::MatrixXs(2, 2*((LandmarkPolyline2D*)landmark)->getNPoints());
            expectedFeature(landmark, expected_features[landmark], expected_features_covs[landmark]);
        }

    // NAIVE FIRST NEAREST NEIGHBOR MATCHING
    LandmarkPolylineMatch* best_match = nullptr;
    FeaturePolyline2D* polyline_feature;
    LandmarkPolyline2D* polyline_landmark;

    auto next_feature_it = polylines_incoming_.begin();
    auto feature_it = next_feature_it++;
    int max_ftr, max_lmk, max_offset, min_offset, offset, from_ftr, from_lmk, to_ftr, to_lmk, N_overlapped;

    // iterate over all polylines features
    while (feature_it != polylines_incoming_.end())
    {
        polyline_feature = (FeaturePolyline2D*)(*feature_it);
        max_ftr = polyline_feature->getNPoints() - 1;
        //std::cout << "feature " << (*feature_it)->id() << ": 0-" << max_ftr << std::endl;
        //std::cout << polyline_feature->getPoints() << std::endl;

        // Check with all landmarks
        for (auto landmark_it = not_matched_landmarks.begin(); landmark_it != not_matched_landmarks.end(); landmark_it++)
        {
            polyline_landmark = (LandmarkPolyline2D*)(*landmark_it);

            // Check all overlapping positions between each feature-landmark pair
            max_lmk = polyline_landmark->getNPoints() - 1;
            max_offset = max_ftr;// - (polyline_landmark->isFirstDefined() || polyline_feature->isLastDefined() ? 0 : 1);
            min_offset = -max_lmk;// + (polyline_feature->isFirstDefined() || polyline_landmark->isLastDefined() ? 0 : 1);

            //std::cout << "\tlandmark " << (*landmark_it)->id() << ": 0-" << max_lmk<< std::endl;
            //std::cout << expected_features[*landmark_it] << std::endl;
            //std::cout << "\tmax_offset " << max_offset << std::endl;
            //std::cout << "\tmin_offset " << min_offset << std::endl;

            // Check all overlapping positions between each feature-landmark pair
            for (offset = min_offset; offset <= max_offset; offset++)
            {
                from_lmk = std::max(0, -offset);
                from_ftr = std::max(0, offset);
                N_overlapped = std::min(max_ftr - from_ftr, max_lmk - from_lmk)+1;
                to_lmk = from_lmk+N_overlapped-1;
                to_ftr = from_ftr+N_overlapped-1;

                //std::cout << "\t\toffset " << offset << std::endl;
                //std::cout << "\t\t\tfrom_lmk " << from_lmk << std::endl;
                //std::cout << "\t\t\tfrom_ftr " << from_ftr << std::endl;
                //std::cout << "\t\t\tN_overlapped " << N_overlapped << std::endl;

                // Compute the squared distance for all overlapped points
                Eigen::ArrayXXd d = (polyline_feature->getPoints().block(0,from_ftr, 2,N_overlapped) -
                                     expected_features[*landmark_it].block(0,from_lmk, 2, N_overlapped)).array();

                Eigen::ArrayXd dist2 = d.row(0).pow(2) + d.row(1).pow(2);
                //std::cout << "\t\t\tsquared distances = " << dist2.transpose() << std::endl;

                if (offset != min_offset && offset != max_offset)
                {
                    // Point-to-line first distance
                    bool from_ftr_not_defined = from_ftr == 0 && !polyline_feature->isFirstDefined();
                    bool from_lmk_not_defined = from_lmk == 0 && !polyline_landmark->isFirstDefined();
                    if (from_ftr_not_defined || from_lmk_not_defined)
                    {
                        //std::cout << "\t\t\tFirst feature not defined distance to line" << std::endl;
                        //std::cout << "\t\t\tA" << expected_features[*landmark_it].col(from_lmk).transpose() << std::endl;
                        //std::cout << "\t\t\tAaux" << expected_features[*landmark_it].col(from_lmk+1).transpose() << std::endl;
                        //std::cout << "\t\t\tB" << polyline_feature->getPoints().col(from_ftr).transpose() << std::endl;
                        dist2(0) = sqDistPointToLine(expected_features[*landmark_it].col(from_lmk),
                                                     expected_features[*landmark_it].col(from_lmk+1),
                                                     polyline_feature->getPoints().col(from_ftr),
                                                     !from_lmk_not_defined,
                                                     !from_ftr_not_defined);
                    }

                    // Point-to-line last distance
                    bool last_ftr_not_defined = !polyline_feature->isLastDefined() && to_ftr == max_ftr;
                    bool last_lmk_not_defined = !polyline_landmark->isLastDefined() && to_lmk == max_lmk;
                    if (last_ftr_not_defined || last_lmk_not_defined)
                    {
                        //std::cout << "\t\t\tLast feature not defined distance to line" << std::endl;
                        //std::cout << "\t\t\tA" << expected_features[*landmark_it].col(to_lmk).transpose() << std::endl;
                        //std::cout << "\t\t\tAaux" << expected_features[*landmark_it].col(to_lmk-1).transpose() << std::endl;
                        //std::cout << "\t\t\tB" << polyline_feature->getPoints().col(to_ftr).transpose() << std::endl;
                        dist2(N_overlapped-1) = sqDistPointToLine(expected_features[*landmark_it].col(to_lmk),
                                                                  expected_features[*landmark_it].col(to_lmk-1),
                                                                  polyline_feature->getPoints().col(to_ftr),
                                                                  !last_lmk_not_defined,
                                                                  !last_ftr_not_defined);
                    }
                }
                //std::cout << "\t\t\tsquared distances = " << dist2.transpose() << std::endl;

                // All squared distances should be witin a threshold
                // Choose the most overlapped one
                if ((dist2 < position_error_th_*position_error_th_).all() && (best_match == nullptr ||
                                                                              (N_overlapped >= best_match->feature_match_to_id_-best_match->feature_match_from_id_+1 &&
                                                                               dist2.mean() < best_match->normalized_score_ )))
                {
                    //std::cout << "BEST MATCH" << std::endl;
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
        // Match found for this feature
        if (best_match != nullptr)
        {
            //std::cout << "\tclosest landmark: " << best_match->landmark_ptr_->id() << std::endl;
            // match
            matches_landmark_from_incoming_[*feature_it] = best_match;
            // move corner feature to output list
            _features_found.splice(_features_found.end(), polylines_incoming_, feature_it);
            // reset match
            best_match = nullptr;
        }
        //else
        //{
        //    std::cout << "\t-------------------------->NO LANDMARK CLOSE ENOUGH!!!!" << std::endl;
        //    std::getchar();
        //}
        feature_it = next_feature_it++;
    }
    return matches_landmark_from_incoming_.size();
}

bool ProcessorTrackerLandmarkPolyline::voteForKeyFrame()
{
    //std::cout << "------------- ProcessorTrackerLandmarkPolyline::voteForKeyFrame:" << std::endl;
    //std::cout << "polylines_last_.size():" << polylines_last_.size()<< std::endl;
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
    //std::cout << "ProcessorTrackerLandmarkPolyline::extractPolylines: " << std::endl;
    // TODO: sort corners by bearing
    std::list<laserscanutils::Polyline> polylines;

    line_finder_.findPolylines(_capture_laser_ptr->getScan(), ((SensorLaser2D*)getSensorPtr())->getScanParams(), polylines);

    for (auto&& pl : polylines)
    {
        //std::cout << "new polyline detected: Points" << std::endl << pl.points_ << std::endl;
        //std::cout << "covs: " << std::endl << pl.covs_ << std::endl;
        _polyline_list.push_back(new FeaturePolyline2D(pl.points_, pl.covs_, pl.first_defined_, pl.last_defined_));
        //std::cout << "new polyline detected: " << std::endl;

        //if (pl.first_defined_ || pl.last_defined_)
        //    std::cout << "FEATURE WITH EXTREME POINT DEFINED!!!!!!!!!" << std::endl;
        //else
        //    std::cout << "feature with extreme points not defined" << std::endl;
    }
    //std::cout << _polyline_list.size() << " polylines extracted" << std::endl;
}

void ProcessorTrackerLandmarkPolyline::expectedFeature(LandmarkBase* _landmark_ptr, Eigen::MatrixXs& expected_feature_,
                                                   Eigen::MatrixXs& expected_feature_cov_)
{
    assert(_landmark_ptr->getType() == LANDMARK_POLYLINE_2D && "ProcessorTrackerLandmarkPolyline::expectedFeature: Bad landmark type");
    LandmarkPolyline2D* polyline_landmark = (LandmarkPolyline2D*)_landmark_ptr;
    assert(expected_feature_.cols() == polyline_landmark->getNPoints() && expected_feature_.rows() == 3 && "ProcessorTrackerLandmarkPolyline::expectedFeature: bad expected_feature_ sizes");

    //std::cout << "ProcessorTrackerLandmarkPolyline::expectedFeature" << std::endl;
    //std::cout << "t_world_sensor_: " << t_world_sensor_.transpose() << std::endl;
    //std::cout << "R_sensor_world_: "  << std::endl << R_sensor_world_ << std::endl;

    expected_feature_ = Eigen::MatrixXs::Zero(3,polyline_landmark->getNPoints());
    expected_feature_cov_ = Eigen::MatrixXs::Zero(2,2*polyline_landmark->getNPoints());
    Eigen::Vector3s col = Eigen::Vector3s::Ones();
    for (auto i = 0; i < polyline_landmark->getNPoints(); i++)
    {
        // ------------ expected feature point
        col.head<2>() = R_sensor_world_ * (polyline_landmark->getPointVector(i) - t_world_sensor_);
        expected_feature_.col(i) = col;

        //std::cout << "Point " << i << std::endl;
        //std::cout << "Landmark global position: " << polyline_landmark->getPointVector(i).transpose() << std::endl;
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

Scalar ProcessorTrackerLandmarkPolyline::sqDistPointToLine(const Eigen::Vector3s& _A, const Eigen::Vector3s& _A_aux,
                                                         const Eigen::Vector3s& _B, bool _A_defined, bool _B_defined)
{
    /* Squared distance from B to the line A_aux-A (match evaluated is A-B)
     *
     * No matter if A_aux is defined or not.
     *
     * Case 1: B not defined
     *
     *      The projection of B over the line AAaux must be in [A_aux, inf(in A direction)). Otherwise, return squared distance Aaux-B.
     *      Check: the angle BAauxA is <= 90º:
     *          (BA)² <= (BAaux)² + (AAaux)²
     *
     *      Case 1.1: A not defined
     *
     *          No more restrictions: return distance line to point.
     *
     *      Case 1.2: A defined
     *
     *          The projection of B over the line AAaux must be in (inf(in Aaux direction), A]. Otherwise, return squared distance A-B.
     *          Additional check: the angle BAAaux is <= 90º:
     *              (BAaux)² <= (BA)² + (AAaux)²
     *
     * Case 2: B is defined and A is not
     *
     *      Returns the distance B-Line if the projection of B to the line is in [A, inf). Otherwise, return squared distance A-B.
     *      Checks if the angle BAAaux is >=  90º: (BAaux)² >= (BA)² + (AAaux)²
     *
     *
     * ( Case B and A are defined is not point-to-line, is point to point -> assertion )
     *
     */

    assert((!_A_defined || !_B_defined) && "ProcessorTrackerLandmarkPolyline::sqDistPointToLine: at least one point must not be defined.");

    Scalar AB_sq = (_B-_A).head<2>().squaredNorm();
    Scalar AauxB_sq = (_B-_A_aux).head<2>().squaredNorm();
    Scalar AAaux_sq = (_A_aux-_A).head<2>().squaredNorm();

    // Case 1
    if (!_B_defined)
    {
        if (AB_sq <= AauxB_sq + AAaux_sq)
        {
            // Case 1.1
            if (!_A_defined)
            {
                //std::cout << "case 1.1" << std::endl;
                return AB_sq - std::pow(((_A_aux-_A).head<2>()).dot((_B-_A).head<2>()),2) / AAaux_sq; //squared distance to line
            }
            // Case 1.2
            else if (AauxB_sq <= AB_sq + AAaux_sq)
            {
                //std::cout << "case 1.2" << std::endl;
                return AB_sq - std::pow(((_A_aux-_A).head<2>()).dot((_B-_A).head<2>()),2) / AAaux_sq; //squared distance to line
            }
        }
    }
    // Case 2
    else if (!_A_defined && _B_defined)
    {
        if (AauxB_sq >= AB_sq + AAaux_sq)
        {
            //std::cout << "case 1.2" << std::endl;
            return AB_sq - std::pow(((_A_aux-_A).head<2>()).dot((_B-_A).head<2>()),2) / AAaux_sq; //squared distance to line
        }
    }

    //std::cout << "return point to point distance" << std::endl;
    // Default return A-B squared distance
    return (_A.head<2>() - _B.head<2>()).squaredNorm();
}

void ProcessorTrackerLandmarkPolyline::createNewLandmarks(LandmarkBaseList& _new_landmarks)
{
    //std::cout << "ProcessorTrackerLandmarkPolyline::createNewLandmarks" << std::endl;
    FeaturePolyline2D* polyline_ft_ptr;
    LandmarkBase* new_lmk_ptr;
    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        new_lmk_ptr = createLandmark(new_feature_ptr);
        //std::cout << "\tfeature: " << new_feature_ptr->id() << std::endl;
        //std::cout << "\tnew_landmark: " << new_lmk_ptr->id() << ": "<< ((LandmarkPolyline2D*)new_lmk_ptr)->getNPoints() << " points" << std::endl;
        _new_landmarks.push_back(new_lmk_ptr);
        // cast
        polyline_ft_ptr = (FeaturePolyline2D*)(new_feature_ptr);
        // create new correspondence
        LandmarkPolylineMatch* match = new LandmarkPolylineMatch();
        match->feature_match_from_id_= 0; // all points match
        match->landmark_match_from_id_ = 0;
        match->feature_match_to_id_= polyline_ft_ptr->getNPoints()-1; // all points match
        match->landmark_match_to_id_ = polyline_ft_ptr->getNPoints()-1;
        match->normalized_score_ = 1.0; // max score
        match->landmark_ptr_ = new_lmk_ptr;
        matches_landmark_from_last_[new_feature_ptr] = match;
    }
}

LandmarkBase* ProcessorTrackerLandmarkPolyline::createLandmark(FeatureBase* _feature_ptr)
{
    assert(_feature_ptr->getType() == FEATURE_POLYLINE_2D);
    //std::cout << "ProcessorTrackerLandmarkPolyline::createLandmark" << std::endl;
    //std::cout << "Robot global pose: " << t_world_robot_.transpose() << std::endl;
    //std::cout << "Sensor global pose: " << t_world_sensor_.transpose() << std::endl;


    FeaturePolyline2D* polyline_ptr = (FeaturePolyline2D*)(_feature_ptr);
    // compute feature global pose
    Eigen::MatrixXs points_global = R_world_sensor_ * polyline_ptr->getPoints().topRows<2>() +
                                    t_world_sensor_ * Eigen::VectorXs::Ones(polyline_ptr->getNPoints()).transpose();

    //std::cout << "Feature local points: " << std::endl << polyline_ptr->getPoints().topRows<2>() << std::endl;
    //std::cout << "Landmark global points: " << std::endl << points_global << std::endl;

    // Create new landmark
    return new LandmarkPolyline2D(points_global, polyline_ptr->isFirstDefined(), polyline_ptr->isLastDefined());
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
    //std::cout << "ProcessorTrackerLandmarkPolyline::establishConstraints" << std::endl;
    LandmarkPolylineMatch* polyline_match;
    FeaturePolyline2D* polyline_feature;
    LandmarkPolyline2D* polyline_landmark;

    for (auto last_feature : *(last_ptr_->getFeatureListPtr()))
    {
        //std::cout << "feature " << last_feature->id() << std::endl;
        polyline_feature = (FeaturePolyline2D*)last_feature;
        polyline_match = (LandmarkPolylineMatch*)matches_landmark_from_last_[last_feature];
        polyline_landmark = (LandmarkPolyline2D*)(polyline_match->landmark_ptr_);

        //std::cout << "match " << std::endl;
        //std::cout << "\tlandmark id " << polyline_match->landmark_ptr_->id() << std::endl;
        //std::cout << "\tfeat from " << polyline_match->feature_match_from_id_ << std::endl;
        //std::cout << "\tfeat to " << polyline_match->feature_match_to_id_ << std::endl;
        //std::cout << "\tland from " << polyline_match->landmark_match_from_id_ << std::endl;
        //std::cout << "\tland to " << polyline_match->landmark_match_to_id_ << std::endl;

        Eigen::MatrixXs points_global = R_world_sensor_ * polyline_feature->getPoints().topRows<2>() +
                                        t_world_sensor_ * Eigen::VectorXs::Ones(polyline_feature->getNPoints()).transpose();
        // GROW LANDMARK
        // Add new front points
        if (polyline_match->feature_match_from_id_ > 0)
        {
            //std::cout << "Add new front points" << std::endl;
            polyline_landmark->addPoints(points_global, // points matrix in global coordinates
                                         polyline_match->feature_match_from_id_-1, // last feature point index to be added
                                         polyline_feature->isFirstDefined(), // is defined
                                         false); // front

            polyline_match->feature_match_from_id_ = 0;
            polyline_match->landmark_match_to_id_ += polyline_match->feature_match_from_id_-1;
        }
        // Change first point
        else if (polyline_match->landmark_match_from_id_ == 0 && !polyline_landmark->isFirstDefined()) // && polyline_match->feature_match_from_id_ == 0
        {
            //std::cout << "Change first point" << std::endl;
            //std::cout << "landmark " << polyline_landmark->id() << ": " << polyline_landmark->getNPoints() << "points" << std::endl;
            //std::cout << "\tpoint 0: " << polyline_landmark->getPointVector(0).transpose() << std::endl;
            //std::cout << "\tpoint 1: " << polyline_landmark->getPointVector(1).transpose() << std::endl;
            //std::cout << "\tfeature point: " << points_global.topLeftCorner(2,1).transpose() << std::endl;

            if (// new defined first
                polyline_feature->isFirstDefined() ||
                // new not defined first
                (points_global.topLeftCorner(2,1)-polyline_landmark->getPointVector(1)).squaredNorm() >
                (points_global.topLeftCorner(2,1)-polyline_landmark->getPointVector(0)).squaredNorm() +
                (polyline_landmark->getPointVector(1)-polyline_landmark->getPointVector(0)).squaredNorm())
                polyline_landmark->setFirst(points_global.leftCols(1), polyline_feature->isFirstDefined());

        }
        // Add back points
        if (polyline_match->feature_match_to_id_ < polyline_feature->getNPoints()-1)
        {
            //std::cout << "Add back points" << std::endl;
            polyline_landmark->addPoints(points_global, // points matrix in global coordinates
                                         polyline_match->feature_match_to_id_+1, // last feature point index to be added
                                         polyline_feature->isLastDefined(), // is defined
                                         true); // back

            polyline_match->feature_match_to_id_ = polyline_feature->getNPoints()-1;
            polyline_match->landmark_match_to_id_ += polyline_match->feature_match_to_id_-1;
        }
        // Change last point
        else if (polyline_match->landmark_match_to_id_ == polyline_landmark->getNPoints()-1 && !polyline_landmark->isLastDefined()) //&& polyline_match->feature_match_to_id_ == polyline_feature->getNPoints()-1
        {
            //std::cout << "Change last point" << std::endl;
            if (// new defined last
                polyline_feature->isLastDefined() ||
                // new not defined last
                (points_global.topRightCorner(2,1)-polyline_landmark->getPointVector(polyline_landmark->getNPoints()-2)).squaredNorm() >
                (points_global.topRightCorner(2,1)-polyline_landmark->getPointVector(polyline_landmark->getNPoints()-1)).squaredNorm() +
                (polyline_landmark->getPointVector(polyline_landmark->getNPoints()-2)-polyline_landmark->getPointVector(polyline_landmark->getNPoints()-1)).squaredNorm())
                polyline_landmark->setLast(points_global.rightCols(1), polyline_feature->isLastDefined());
        }

        // ESTABLISH CONSTRAINTS
        //std::cout << "ESTABLISH CONSTRAINTS" << std::endl;

        // Constraints for all inner and defined feature points
        unsigned int offset_id = polyline_match->landmark_match_from_id_ - polyline_match->feature_match_from_id_;
        for (unsigned int i = 0; i < polyline_feature->getNPoints(); i++)
        {
            // First not defined point
            if (i == 0 && !polyline_feature->isFirstDefined())
                // last point to line constraint
                last_feature->addConstraint(new ConstraintPointToLine2D(polyline_feature, polyline_landmark, i, i+offset_id, i+offset_id+1));

            // Last not defined point
            else if (i == polyline_feature->getNPoints()-1 && !polyline_feature->isLastDefined())
                // last point to line constraint
                last_feature->addConstraint(new ConstraintPointToLine2D(polyline_feature, polyline_landmark, i, i+offset_id, i+offset_id-1));

            // Defined point
            else
                // point to point constraint
                last_feature->addConstraint(new ConstraintPoint2D(polyline_feature, polyline_landmark, i, i+offset_id));
        }
    }
}

ConstraintBase* ProcessorTrackerLandmarkPolyline::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
    // not used
    return nullptr;
}

ProcessorBase* ProcessorTrackerLandmarkPolyline::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorParamsPolyline* params = (ProcessorParamsPolyline*)_params;
    ProcessorTrackerLandmarkPolyline* prc_ptr = new ProcessorTrackerLandmarkPolyline(params->line_finder_params_, params->new_features_th, params->loop_frames_th);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
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
