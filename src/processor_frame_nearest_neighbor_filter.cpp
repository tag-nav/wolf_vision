#include "processor_frame_nearest_neighbor_filter.h"

namespace wolf
{

ProcessorFrameNearestNeighborFilter::ProcessorFrameNearestNeighborFilter(const Params& _params):
    ProcessorLoopClosureBase("FRAME NEAREST NEIGHBOR FILTER", _params.time_tolerance),
    params_(_params)
{
  // area of ellipse based on the Chi-Square Probabilities
  // https://people.richland.edu/james/lecture/m170/tbl-chi.html
  // cover both 2D and 3D cases

  if(params_.distance_type_ == DistanceType::LC_POINT_ELLIPSE ||
     params_.distance_type_ == DistanceType::LC_ELLIPSE_ELLIPSE)
  {
    switch ((int)(params_.probability_*100))
    {
    case 900:  area_ = 4.605; // 90% probability
      break;
    case 950:  area_ = 5.991; // 95% probability
      break;
    case 990:  area_ = 9.210; // 99% probability
      break;
    default :  area_ = 5.991; // 95% probability
      break;
    }
  }
  if (params_.distance_type_ == DistanceType::LC_POINT_ELLIPSOID ||
      params_.distance_type_ == DistanceType::LC_ELLIPSOID_ELLIPSOID)
  {
    switch ((int)(params_.probability_*100))
    {
    case 900:  area_ = 6.251;  // 90% probability
      break;
    case 950:  area_ = 7.815;  // 95% probability
      break;
    case 990:  area_ = 11.345; // 99% probability
      break;
    default :  area_ = 7.815;  // 95% probability
      break;
    }
  }

  /*
   * The area is based on the Chi-Square Probabilities
   * Becasue they are very big for high likelihood, it is here manually set
   * on ONE. Therefore the ellipses/ ellipsoids are based on the unit ellipse/
   * ellipsoids and the axis are scaled by a / b / c
   */

  area_ = 1;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::findCandidates(const CaptureBasePtr& /*_incoming_ptr*/)
{
  const ProblemPtr problem_ptr = getProblem();

  const std::string frame_structure =
      problem_ptr->getTrajectoryPtr()->getFrameStructure();

  // get the list of all frames
  const FrameBaseList& frame_list = problem_ptr->
                                    getTrajectoryPtr()->
                                    getFrameList();

  bool found_possible_candidate = false;

  for (const FrameBasePtr& key_it : frame_list)
  {
    // check for LC just if frame is key frame
    // Assert that the evaluated KF has a capture of the
    // same sensor as this processor
    if (key_it->isKey() && key_it->getCaptureOf(getSensorPtr()/*, "LASER 2D"*/) != nullptr)
    {
      // Check if the two frames currently evaluated are already
      // constrained one-another.
      const ConstraintBaseList& ctr_list = key_it->getConstrainedByList();

      bool are_constrained = false;
      for (const ConstraintBasePtr& crt : ctr_list)
      {
        // Are the two frames constrained one-another ?
        if (crt->getFrameOtherPtr() == problem_ptr->getLastKeyFramePtr())
        {
          // By this very processor ?
          if (crt->getProcessor() == shared_from_this())
          {
            WOLF_DEBUG("Frames are already constrained together.");
            are_constrained = true;
            break;
          }
        }
      }
      if (are_constrained) continue;

      // check if feame is potential candidate for loop closure with
      // chosen distance type
      switch (params_.distance_type_)
      {
      case LoopclosureDistanceType::LC_POINT_ELLIPSE:
      {
        if (frame_structure == "PO 3D" || frame_structure == "POV 3D") // @todo add frame structure "PQVBB 3D"
        {
          WOLF_ERROR("Distance Type LC_POINT_ELLIPSE does not fit 3D frame structure");
          return false;
        }

        Eigen::Vector5s frame_covariance;
        if (!computeEllipse2D(key_it, frame_covariance)) continue;

        const Eigen::VectorXs current_position = getProblem()->getCurrentState();
        found_possible_candidate = point2DIntersect(current_position,
                                                    frame_covariance);
        break;
      }

      case LoopclosureDistanceType::LC_ELLIPSE_ELLIPSE:
      {
        if (frame_structure == "PO 3D" || frame_structure == "POV 3D")
        {
          WOLF_ERROR("Distance Type LC_ELLIPSE_ELLIPSE does not fit 3D frame structure");
          return false;
        }

        Eigen::Vector5s frame_covariance, current_covariance;
        if (!computeEllipse2D(key_it,
                              frame_covariance)) continue;
        if (!computeEllipse2D(getProblem()->getLastKeyFramePtr(),
                              current_covariance)) continue;
        found_possible_candidate = ellipse2DIntersect(frame_covariance,
                                                      current_covariance);
        break;
      }

      case LoopclosureDistanceType::LC_POINT_ELLIPSOID:
      {
        if (frame_structure == "PO 2D")
        {
          WOLF_ERROR("Distance Type POINT_ELLIPSOID does not fit 2D frame structure");
          return false;
        }
        Eigen::Vector10s frame_covariance;
        if (!computeEllipsoid3D(key_it,
                                frame_covariance)) continue;
        const Eigen::VectorXs current_position = getProblem()->getCurrentState();
        found_possible_candidate = point3DIntersect(current_position,
                                                    frame_covariance);
        break;
      }

      case LoopclosureDistanceType::LC_ELLIPSOID_ELLIPSOID:
      {
        if (frame_structure == "PO 2D")
        {
          WOLF_ERROR("Distance Type ELLIPSOID_ELLIPSOID does not fit 2D frame structure");
          return false;
        }
        Eigen::Vector10s frame_covariance, current_covariance;
        if (!computeEllipsoid3D(key_it,
                                frame_covariance)) continue;
        if (!computeEllipsoid3D(getProblem()->getLastKeyFramePtr(),
                                frame_covariance)) continue;
        found_possible_candidate = ellipsoid3DIntersect(frame_covariance,
                                                        current_covariance);
        break;
      }

      case LoopclosureDistanceType::LC_MAHALANOBIS_DISTANCE:
      {
        found_possible_candidate = insideMahalanisDistance(key_it,
                                             problem_ptr->getLastKeyFramePtr());
        break;
      }

      default:
        WOLF_WARN("NO IMPLEMENTATION FOR OTHER DISTANCE CALCULATION TYPES YET.");
        found_possible_candidate = false;
        return false;
      }

      // if a candidate was detected, push it to the appropiate list
      if (found_possible_candidate)
      {
        if (!frameInsideBuffer(key_it))
          loop_closure_candidates.push_back(key_it);
        else
          close_candidates.push_back(key_it);
      }

    } // end if key_it is keyframe
  } // end loop through every frame in list

  return found_possible_candidate;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::computeEllipse2D(const FrameBasePtr& frame_ptr,
                                                           Eigen::Vector5s& ellipse)
{
  // get 3x3 covariance matrix AKA variance
  const Eigen::MatrixXs covariance =
      getProblem()->getFrameCovariance(frame_ptr);

  if (!isCovariance(covariance))
  {
    WOLF_WARN("Covariance is not proper !");
    return false;
  }

  // get position of frame AKA mean [x, y]
  const Eigen::VectorXs frame_position  = frame_ptr->getPPtr()->getState();
  ellipse(0) = frame_position(0);
  ellipse(1) = frame_position(1);

  // compute covariance error ellipse around state
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2s> solver(covariance.block(0,0,2,2));
  Scalar eigenvalue1 = solver.eigenvalues()[0];     // smaller value
  Scalar eigenvalue2 = solver.eigenvalues()[1];     // bigger value
  //Eigen::VectorXs eigenvector1 = solver.eigenvectors().col(0);
  Eigen::VectorXs eigenvector2 = solver.eigenvectors().col(1);

  const Scalar scale = std::sqrt(area_);
  ellipse(2) = scale * std::sqrt(eigenvalue2) / 2.;          // majorAxis
  ellipse(3) = scale * std::sqrt(eigenvalue1) / 2.;          // minorAxis
  ellipse(4) = std::atan2(eigenvector2[1], eigenvector2[0]);  // tilt

  if (ellipse(4) < Scalar(0)) ellipse(4) += Scalar(2) * M_PI;

  // [ pos_x, pos_y, a, b, tilt]
  return true;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::computeEllipsoid3D(const FrameBasePtr& frame_pointer,
                                                             Eigen::Vector10s& ellipsoid)
{
  // Ellipse description [ pos_x, pos_y, pos_z, a, b, c, quat_w, quat_z, quat_y, quat_z]

  // get position of frame AKA mean [x, y, z]
  const Eigen::VectorXs frame_position  = frame_pointer->getPPtr()->getState();
  ellipsoid(0) = frame_position(0);
  ellipsoid(1) = frame_position(1);
  ellipsoid(2) = frame_position(2);

  // get 9x9 covariance matrix AKA variance
  const Eigen::MatrixXs covariance = getProblem()->getFrameCovariance(frame_pointer);

  if (!isCovariance(covariance)) return false;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3s> solver(covariance.block(0,0,3,3));
  const Scalar eigenvalue1 = solver.eigenvalues()[0];     // smaller value
  const Scalar eigenvalue2 = solver.eigenvalues()[1];     // mediate value
  const Scalar eigenvalue3 = solver.eigenvalues()[2];     // bigger value

  const Scalar scale = std::sqrt(area_);

  ellipsoid(3) = scale * std::sqrt(std::abs(eigenvalue3)) / Scalar(2); // majorAxis
  ellipsoid(4) = scale * std::sqrt(std::abs(eigenvalue2)) / Scalar(2); // mediumAxis
  ellipsoid(5) = scale * std::sqrt(std::abs(eigenvalue1)) / Scalar(2); // minorAxis

  // ROTATION COMPUTATION get rotation of the three axis from eigenvector
  // eigenvector belonging to biggest eigenvalue gives direction / z_axis
  // of ellipsoid

  // get eigenvectors that correspond to eigenvalues
  //const Eigen::Vector3s eigenvector1 = solver.eigenvectors().col(0); // minorAxis
  const Eigen::Vector3s eigenvector2 = solver.eigenvectors().col(1); // mediumAxis ->y
  const Eigen::Vector3s eigenvector3 = solver.eigenvectors().col(2); // majorAxis -> x

  // computed axis of coordinate system in ellipsoid center
  const Eigen::Vector3s z_axis = eigenvector3.cross(eigenvector2);
  const Eigen::Vector3s y_axis = z_axis.cross(eigenvector3);

  // use them to fill rotation matrix
  Eigen::Matrix3s rotationMatrix;
  rotationMatrix.col(0) = eigenvector3.normalized();
  rotationMatrix.col(1) = y_axis.normalized();
  rotationMatrix.col(2) = z_axis.normalized();

  // get quaternions for transformation
  Eigen::Quaternions rotation(rotationMatrix);
  rotation.normalize();

  ellipsoid(6) = rotation.w();
  ellipsoid(7) = rotation.x();
  ellipsoid(8) = rotation.y();
  ellipsoid(9) = rotation.z();

  WOLF_DEBUG("made ellipsoid: \n[", ellipsoid.transpose(), "]");

  // [ pos_x, pos_y, pos_z, a, b, c, alpha, beta, gamma]
  return true;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::ellipse2DIntersect(const Eigen::VectorXs& ellipse1,
                                                             const Eigen::VectorXs& ellipse2)
{
  for (int i = 0 ; i < 360 ; i += params_.sample_step_degree_)
  {
    // parameterized form of first ellipse gives point on first ellipse
    // https://www.uwgb.edu/dutchs/Geometry/HTMLCanvas/ObliqueEllipses5.HTM
    Eigen::VectorXs pointOnEllipse(2);
    const Scalar theta = Scalar(i) * M_PI / 180.0;

    pointOnEllipse(0) = ellipse1(0) +
                          ellipse1(2) * std::cos(ellipse1(4)) * std::cos(theta) -
                          ellipse1(3) * std::sin(ellipse1(4)) * std::sin(theta);

    pointOnEllipse(1) = ellipse1(1) +
                          ellipse1(2) * std::sin(ellipse1(4)) * std::cos(theta) +
                          ellipse1(3) * std::cos(ellipse1(4)) * std::sin(theta);

    //WOLF_DEBUG(" for ", i, " deg / ", theta " rad --->   [" ,
    //            pointOnEllipse.transpose(), "]");

    // check if point lies inside the other ellipse
    if (point2DIntersect(pointOnEllipse, ellipse2)) return true;
  }

  return false;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::point2DIntersect(const Eigen::VectorXs& point,
                                                           const Eigen::VectorXs& ellipse)
{
  const Scalar area51 = std::pow( ((point(0) - ellipse(0)) * cos(ellipse(4))
                                  + (point(1) - ellipse(1)) * sin(ellipse(4))), 2 )
                        / std::pow( ellipse(2), 2 )
                        + std::pow( ((point(0) - ellipse(0)) * sin(ellipse(4))
                                    - (point(1) - ellipse(1)) * cos(ellipse(4))), 2 )
                        / std::pow( ellipse(3), 2 );

  //WOLF_DEBUG("computed area = ", area51);

  if ( area51 - area_ <=  0) return true;

  return false;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::ellipsoid3DIntersect(const Eigen::VectorXs &ellipsoid1,
                                                               const Eigen::VectorXs &ellipsoid2)
{
  // get transformation from elliposiod1 center frame to world frame
  Eigen::Matrix4s transformation_matrix;
  Eigen::Quaternions rotation(ellipsoid1(6),ellipsoid1(7),
                              ellipsoid1(8),ellipsoid1(9));
//  Eigen::Vector4s translation;
//  translation << ellipsoid1(0), ellipsoid1(1), ellipsoid1(2), 1;
  transformation_matrix.block(0,0,3,3) = rotation.toRotationMatrix();
  transformation_matrix.col(3) << ellipsoid1(0), ellipsoid1(1), ellipsoid1(2), 1;

  Eigen::Vector4s point_hom = Eigen::Vector4s::Constant(1);

  Scalar omega = 0;
  for (int i = 0 ; i < 360 ; i += params_.sample_step_degree_)
  {
    const Scalar theta = Scalar(i) * M_PI / 180.0;
    for( int j = 0 ; j < 180 ; j += params_.sample_step_degree_)
    {
      omega = Scalar(j) * M_PI / 180.0;

      // compute point on surface of first ellipsoid
      point_hom(0) = ellipsoid1(3) * std::cos(theta) * std::sin(omega);
      point_hom(1) = ellipsoid1(4) * std::sin(theta) * std::sin(omega);
      point_hom(2) = ellipsoid1(5) * std::cos(omega);

      // transform point into world frame
      const Eigen::Vector4s point_on_ellipsoid = transformation_matrix * point_hom;

      // check if 3D point lies inside the second ellipsoid
      if (point3DIntersect(point_on_ellipsoid, ellipsoid2))
        return true;
    }
  }
  return false;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::point3DIntersect(const Eigen::VectorXs &point,
                                                           const Eigen::VectorXs &ellipsoid)
{
  // get transformation from ellipsoid center frame to world frame
  Eigen::Matrix4s transformation_matrix = Eigen::Matrix4s::Identity();

  Eigen::Quaternions rotation(ellipsoid(6),ellipsoid(7),
                              ellipsoid(8),ellipsoid(9));
//  Eigen::Vector4s translation;
//  translation << ellipsoid(0), ellipsoid(1), ellipsoid(2), 1;
  transformation_matrix.block(0,0,3,3) = rotation.toRotationMatrix();
  transformation_matrix.col(3) << ellipsoid(0), ellipsoid(1), ellipsoid(2), 1;
  // inverse to get transformation from world frame to ellipsoid center frame
  transformation_matrix = transformation_matrix.inverse().eval();

  // homogenize 3D point
  // ???
  Eigen::Vector4s point_hom;
  point_hom << point(0), point(1), point(2), 1;

  // transform point from world frame to elliposiod center frame
  Eigen::Vector4s transformed_point = transformation_matrix * point_hom;

  // check if point is inside ellipsoid with general equation
  Scalar area51 = std::pow(transformed_point(0), 2) / std::pow( ellipsoid(3), 2) +
      std::pow(transformed_point(1), 2) / std::pow(ellipsoid(4), 2) +
      std::pow(transformed_point(2), 2) / std::pow(ellipsoid(5), 2);

  WOLF_DEBUG("computed area = ", area51);

  if (area51 - area_ <=  0) return true;

  return false;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::insideMahalanisDistance(const FrameBasePtr& trajectory_frame,
                                                                  const FrameBasePtr& query_frame)
{
  const Scalar distance = MahalanobisDistance(trajectory_frame, query_frame);

  WOLF_DEBUG("Mahalanobis Distance = ", distance);

  if ( distance < 0 )
  {
    WOLF_DEBUG("NO COVARIANCE AVAILABLE");
  }

  /*
     * TODO:
     * check if distance is smaller than a threshold,
     * if yes -> return true
     * else -> return false
     *
     */
  return false;
}

//##############################################################################
Scalar ProcessorFrameNearestNeighborFilter::MahalanobisDistance(const FrameBasePtr& trajectory,
                                                                const FrameBasePtr& query)
{
  Scalar distance = -1;
  Eigen::VectorXs traj_pose, query_pose;

  // get state and covariance matrix for both frames
  if (trajectory->getPPtr()->getState().size() == 2)
  {
    traj_pose.resize(3);
    query_pose.resize(3);
  }
  else
  {
    traj_pose.resize(7);
    query_pose.resize(7);
  }

  traj_pose << trajectory->getPPtr()->getState(),
               trajectory->getOPtr()->getState();

  query_pose << query->getPPtr()->getState(),
                query->getOPtr()->getState();

  const Eigen::MatrixXs traj_covariance  = getProblem()->getFrameCovariance(trajectory);
  const Eigen::MatrixXs query_covariance = getProblem()->getFrameCovariance(query);

  if ( !isCovariance(traj_covariance) || !isCovariance(query_covariance))
    return distance;

  const Eigen::MatrixXs covariance = traj_covariance * query_covariance.transpose();

  const Eigen::VectorXs pose_difference = traj_pose - query_pose;
  distance = pose_difference.transpose() * covariance * pose_difference;
  distance = std::sqrt(distance);

  return distance;
}

//##############################################################################
bool ProcessorFrameNearestNeighborFilter::frameInsideBuffer(const FrameBasePtr& frame_ptr)
{
  FrameBasePtr keyframe = getProblem()->getLastKeyFramePtr();
  if ( (int)frame_ptr->id() < ( (int)keyframe->id() - params_.buffer_size_ ))
    return false;
  else
    return true;
}

} // namespace wolf

