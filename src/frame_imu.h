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
          /** \brief Constructor of non-key Frame with only time stamp
           *
           * Constructor with only time stamp
           * \param _ts is the time stamp associated to this frame, provided in seconds
           * \param _p_ptr StateBlock pointer to the position (default: nullptr)
           * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
           * \param _o_ptr StateBlock pointer to the orientation (default: nullptr). Pass a StateQuaternion if needed.
           * \param _ba_ptr StateBlock pointer to the acceleration bias (default: nullptr).
           * \param _bg_ptr StateBlock pointer to the gyrometer bias (default: nullptr).
           **/
        FrameIMU(const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _v_ptr = nullptr, StateQuaternionPtr _q_ptr =
                         nullptr,
                 StateBlockPtr _ba_ptr = nullptr, StateBlockPtr _bg_ptr = nullptr);

          /** \brief Constructor with type, time stamp and state pointer
           *
           * Constructor with type, time stamp and state pointer
           * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
           * \param _ts is the time stamp associated to this frame, provided in seconds
           * \param _p_ptr StateBlock pointer to the position (default: nullptr)
           * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
           * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
           * \param _ba_ptr StateBlock pointer to the acceleration bias (default: nullptr).
           * \param _bg_ptr StateBlock pointer to the gyrometer bias (default: nullptr).
           **/
        FrameIMU(const FrameKeyType& _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _v_ptr = nullptr,
                 StateQuaternionPtr _q_ptr = nullptr, StateBlockPtr _ba_ptr = nullptr, StateBlockPtr _bg_ptr = nullptr);

          /** \brief Constructor with type, time stamp and state vector
          * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
          * \param _ts is the time stamp associated to this frame, provided in seconds
          * \param _x state vector of size 16, organized as [position, quaternion, velocity, acc_bias, gyro_bias]
          **/
         FrameIMU(const FrameKeyType & _tp, const TimeStamp& _ts, const Eigen::VectorXs& _x);
          virtual ~FrameIMU();

          // Frame values ------------------------------------------------

          StateBlockPtr getAccBiasPtr() const;
          StateBlockPtr getGyroBiasPtr() const;

          void setState(const Eigen::VectorXs& _st);
          Eigen::VectorXs getState() const;
          void getState(Eigen::VectorXs& state) const;

      private:
          /** \brief Sets the Frame status (see wolf.h for Frame status)
           **/
          void setStatus(StateStatus _st);
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
