#ifndef FRAME_IMU_H_
#define FRAME_IMU_H_

// Fwd refs
namespace wolf{
class TrajectoryBase;
class CaptureBase;
class StateBlock;
class StateQuaternion;
}

//Wolf includes
#include "frame_base.h"


namespace wolf {

  class FrameIMU : public FrameBase
  {
      public:
          typedef std::shared_ptr<FrameIMU> Ptr;
          typedef std::weak_ptr<FrameIMU> WPtr;

      public:

          /** \brief Constructor with type, time stamp and state vector
          * \param _tp indicates frame type. Either NON_KEY_FRAME or KEY_FRAME. (types defined in wolf.h)
          * \param _ts is the time stamp associated to this frame, provided in seconds
          * \param _x state vector of size 16, organized as [position, quaternion, velocity, acc_bias, gyro_bias]
          **/
         FrameIMU(const FrameType & _tp, const TimeStamp& _ts, const Eigen::VectorXs& _x);
          virtual ~FrameIMU();

          // State blocks ------------------------------------------------

          StateBlockPtr getAccBiasPtr() const;
          StateBlockPtr getGyroBiasPtr() const;

      public:
          static FrameBasePtr create(const FrameType & _tp,
                                     const TimeStamp& _ts,
                                     const Eigen::VectorXs& _x = Eigen::VectorXs::Zero(16));
};

// IMPLEMENTATION //

inline StateBlockPtr FrameIMU::getAccBiasPtr() const
{
    return getStateBlockPtr(3);
}

inline StateBlockPtr FrameIMU::getGyroBiasPtr() const
{
    return getStateBlockPtr(4);
}

} // namespace wolf

#endif
