#ifndef _WOLF_SRC_PROCESSOR_FRAME_NEAREST_NEIGHBOR_FILTER_H
#define _WOLF_SRC_PROCESSOR_FRAME_NEAREST_NEIGHBOR_FILTER_H

// Wolf related headers
#include "base/processor/processor_loopclosure_base.h"
#include "base/utils/params_server.hpp"
#include "base/state_block/state_block.h"

namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsFrameNearestNeighborFilter)
WOLF_PTR_TYPEDEFS(ProcessorFrameNearestNeighborFilter)

enum class LoopclosureDistanceType : std::size_t
{
  LC_POINT_ELLIPSE = 1,       // 2D
  LC_ELLIPSE_ELLIPSE,         // 2D
  LC_POINT_ELLIPSOID,         // 3D
  LC_ELLIPSOID_ELLIPSOID,     // 3D
  LC_MAHALANOBIS_DISTANCE     // 2D & 3D
};

struct ProcessorParamsFrameNearestNeighborFilter : public ProcessorParamsLoopClosure
{
  using DistanceType = LoopclosureDistanceType;

  ProcessorParamsFrameNearestNeighborFilter() :
    buffer_size_(10),
    sample_step_degree_(10),
    distance_type_(LoopclosureDistanceType::LC_POINT_ELLIPSE),
    probability_(5.991)  { }

  ProcessorParamsFrameNearestNeighborFilter(
      const ProcessorParamsFrameNearestNeighborFilter& o) :
    buffer_size_(o.buffer_size_),
    sample_step_degree_(o.sample_step_degree_),
    distance_type_(o.distance_type_),
    probability_(o.probability_)
  {
    //
  }
    ProcessorParamsFrameNearestNeighborFilter(std::string _unique_name, const paramsServer& _server):
        ProcessorParamsLoopClosure(_unique_name, _server)
    {
        buffer_size_ = _server.getParam<int>(_unique_name + "/buffer_size");
        sample_step_degree_ = _server.getParam<int>(_unique_name + "/sample_step_degree");
        auto distance_type_str = _server.getParam<std::string>(_unique_name + "/distance_type");
        if(distance_type_str.compare("LC_POINT_ELLIPSE")) distance_type_ = LoopclosureDistanceType::LC_POINT_ELLIPSE;
        else if(distance_type_str.compare("LC_ELLIPSE_ELLIPSE")) distance_type_ = LoopclosureDistanceType::LC_ELLIPSE_ELLIPSE;
        else if(distance_type_str.compare("LC_POINT_ELLIPSOID")) distance_type_ = LoopclosureDistanceType::LC_POINT_ELLIPSOID;
        else if(distance_type_str.compare("LC_ELLIPSOID_ELLIPSOID")) distance_type_ = LoopclosureDistanceType::LC_ELLIPSOID_ELLIPSOID;
        else if(distance_type_str.compare("LC_MAHALANOBIS_DISTANCE")) distance_type_ = LoopclosureDistanceType::LC_MAHALANOBIS_DISTANCE;
        else throw std::runtime_error("Failed to fetch a valid value for the enumerate LoopclosureDistanceType. Value provided: " + distance_type_str);
        probability_ = _server.getParam<Scalar>(_unique_name + "/probability");
    }
  virtual ~ProcessorParamsFrameNearestNeighborFilter() = default;

  int buffer_size_;
  int sample_step_degree_;
  DistanceType distance_type_;
  Scalar probability_;
};

class ProcessorFrameNearestNeighborFilter : public ProcessorLoopClosureBase
{
private:

  // area of the computed covariance ellipse.
  // depends on how many percent of data should be considered.
  Scalar area_;

  ProcessorParamsFrameNearestNeighborFilterPtr params_NNF;

public:

  using Params    = ProcessorParamsFrameNearestNeighborFilter;
  using ParamsPtr = ProcessorParamsFrameNearestNeighborFilterPtr;
  using DistanceType = Params::DistanceType;

  ProcessorFrameNearestNeighborFilter(ParamsPtr _params_NNF);
  virtual ~ProcessorFrameNearestNeighborFilter() = default;
  virtual void configure(SensorBasePtr _sensor) { };

  inline DistanceType getDistanceType() const noexcept {return params_NNF->distance_type_;}

protected:

  virtual bool findCandidates(const CaptureBasePtr& _incoming_ptr);

  // returns Ellipse in 2D case [ pos_x, pos_y, a, b, tilt]
  bool computeEllipse2D(const FrameBasePtr& frame_ptr,
                        Eigen::Vector5s& ellipse);

  // returns Ellipsoid in 3D case
  // [ pos_x, pos_y, pos_z, a, b, c, quat_w, quat_z, quat_y, quat_z]
  bool computeEllipsoid3D(const FrameBasePtr& frame_ptr,
                          Eigen::Vector10s& ellipsoid);

  // returns true if the two 2D ellipses intersect
  bool ellipse2DIntersect(const Eigen::VectorXs &ellipse1,
                          const Eigen::VectorXs &ellipse2);

  // returns true if a 2D point lies inside a 2D ellipse
  bool point2DIntersect(const Eigen::VectorXs &point,
                        const Eigen::VectorXs &ellipse);

  // returns true if the two 3D ellipsoids intersect
  bool ellipsoid3DIntersect(const Eigen::VectorXs &ellipsoid1,
                            const Eigen::VectorXs &ellipsoid2);

  // returns true if a 3D point lies inside a 3D ellipsoid
  bool point3DIntersect(const Eigen::VectorXs &point,
                        const Eigen::VectorXs &ellipsoid);

  // returns true if frame lies within Mahalanobis Distance
  bool insideMahalanisDistance(const FrameBasePtr& trajectory_frame,
                               const FrameBasePtr& query_frame);

  // computes the Mahalanobis Distance
  Scalar MahalanobisDistance(const FrameBasePtr& trajectory_frame,
                             const FrameBasePtr& query_frame);

  bool frameInsideBuffer(const FrameBasePtr& frame_ptr);
};

} // namespace wolf

#endif // _WOLF_SRC_PROCESSOR_FRAME_NEAREST_NEIGHBOR_FILTER_H_
