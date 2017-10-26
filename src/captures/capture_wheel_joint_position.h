/**
 * \file diff_drive_tools.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef CAPTURE_WHEEL_JOINT_POSITION_H_
#define CAPTURE_WHEEL_JOINT_POSITION_H_

//wolf includes
#include "../capture_motion.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(CaptureWheelJointPosition)

/**
 * @brief The CaptureWheelJointPosition class
 *
 * Represents a list of wheel positions in radian.
 */
class CaptureWheelJointPosition : public CaptureMotion
{
protected:

  using NodeBase::node_type_;

public:

  /**
   * \brief Constructor
   **/
  CaptureWheelJointPosition(const TimeStamp& _ts,
                            const SensorBasePtr& _sensor_ptr,
                            const Eigen::VectorXs& _positions,
                            FrameBasePtr _origin_frame_ptr);

  CaptureWheelJointPosition(const TimeStamp& _ts,
                            const SensorBasePtr& _sensor_ptr,
                            const Eigen::VectorXs& _positions,
                            const Eigen::MatrixXs& _positions_cov,
                            FrameBasePtr _origin_frame_ptr,
                            StateBlockPtr _p_ptr = nullptr,
                            StateBlockPtr _o_ptr = nullptr,
                            StateBlockPtr _intr_ptr = nullptr);

  virtual ~CaptureWheelJointPosition() = default;

  virtual VectorXs correctDelta(const VectorXs& _delta,
                                const VectorXs& _delta_error) override;

  const Eigen::VectorXs& getPositions() const;

  const Eigen::MatrixXs& getPositionsCov() const;

protected:

  Eigen::VectorXs positions_;
  Eigen::MatrixXs positions_cov_;
};


/// @todo Enforce some logic on the wheel joint pos data

//template <typename E>
//constexpr typename std::underlying_type<E>::type val(E&& e) noexcept
//{
//  return static_cast<typename std::underlying_type<E>::type>(std::forward<E>(e));
//}

//template <std::size_t N>
//struct NumWheelTraits
//{
//  static constexpr std::size_t num_wheel = N;

//  struct Positions
//  {
//    using data_t = Eigen::Matrix<Scalar, num_wheel, 1>;
//  };
//};

//template <typename Derived>
//struct MobileBaseControllerTraits
//{
//  using controller_t = Derived;

//  static constexpr decltype(Derived::num_wheel) num_wheel = Derived::num_wheel;

//  using wheel_index_t = typename Derived::WheelsIndexes;

//  MobileBaseControllerTraits()
//  {
//    static_assert(true, "");
//  }
//};

//struct DiffDriveController : NumWheelTraits<2>
//{
//  enum class WheelsIndexes : std::size_t
//  {
//    Left  = 0,
//    Right = 1
//  };

//  class Positions
//  {
//    Eigen::Matrix<Scalar, num_wheel, 1> values_;

//  public:

//    Positions(const Scalar left_wheel_value,
//              const Scalar rigth_wheel_value) :
//    values_(Eigen::Matrix<Scalar, num_wheel, 1>(left_wheel_value, rigth_wheel_value))
//    { }
//  };
//};

//struct DiffDriveFourWheelsController : NumWheelTraits<4>
//{
//  enum class WheelsIndexes : std::size_t
//  {
//    Front_Left  = 0,
//    Front_Right = 1,
//    Rear_Left   = 2,
//    Rear_Right  = 3
//  };
//};

///**
// * @brief The CaptureWheelJointPosition class
// *
// * Represents a list of wheel positions.
// */
//template <typename ControllerType>
//class CaptureWheelJointPosition final :
//    public CaptureBase, MobileBaseControllerTraits<ControllerType>
//{
//public:

//  using MobileBaseControllerTraits<ControllerType>::controller_t;
//  using typename MobileBaseControllerTraits<ControllerType>::wheel_index_t;
//  using MobileBaseControllerTraits<ControllerType>::num_wheel;

//  /**
//   * \brief Constructor with ranges
//   **/
//  CaptureWheelJointPosition(const TimeStamp& _ts,
//                            const SensorBasePtr& _sensor_ptr,
//                            const Eigen::Matrix<Scalar, num_wheel, 1>& _positions) :
//    CaptureBase("WHEELS POSITION", _ts, _sensor_ptr),
//    positions_(_positions)
//  {
//    //
//  }

//  ~CaptureWheelJointPosition() = default;

////  void setSensorPtr(const SensorBasePtr sensor_ptr) override;

//  std::size_t getNumWheels() const noexcept
//  {
//    return num_wheel;
//  }

//  template <wheel_index_t wheel_index>
//  const Scalar& getPosition() const
//  {
//    return positions_(val(wheel_index));
//  }

//protected:

//  Eigen::Matrix<Scalar, num_wheel, 1> positions_;

//  //SensorLaser2DPtr laser_ptr_; //specific pointer to sensor laser 2D object
//};

//using CaptureDiffDriveWheelJointPosition = wolf::CaptureWheelJointPosition<wolf::DiffDriveController>;


} // namespace wolf

#endif /* CAPTURE_WHEEL_JOINT_POSITION_H_ */
