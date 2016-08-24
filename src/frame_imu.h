#ifndef FRAME_IMU_H_
#define FRAME_IMU_H_

// Fwd refs
namespace wolf{
class TrajectoryBase;
class CaptureBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "frame_base.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "node_constrained.h"


namespace wolf {

  class FrameIMU : public FrameBase
  {
      protected:
          StateBlock* acc_bias_ptr_;      ///< Accleration bias state block pointer
          StateBlock* gyro_bias_ptr_;      ///< Gyrometer bias state block pointer
      public:
          /** \brief Constructor of non-key Frame with only time stamp
           *
           * Constructor with only time stamp
           * \param _ts is the time stamp associated to this frame, provided in seconds
           * \param _p_ptr StateBlock pointer to the position (default: nullptr)
           * \param _o_ptr StateBlock pointer to the orientation (default: nullptr). Pass a StateQuaternion if needed.
           * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
           * \param _ba_ptr StateBlock pointer to the acceleration bias (default: nullptr).
           * \param _bg_ptr StateBlock pointer to the gyrometer bias (default: nullptr).
           **/
          FrameIMU(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr, StateBlock* _ba_ptr = nullptr, StateBlock* _bg_ptr = nullptr);

          /** \brief Constructor with type, time stamp and state pointer
           *
           * Constructor with type, time stamp and state pointer
           * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
           * \param _ts is the time stamp associated to this frame, provided in seconds
           * \param _p_ptr StateBlock pointer to the position (default: nullptr)
           * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
           * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
           * \param _ba_ptr StateBlock pointer to the acceleration bias (default: nullptr).
           * \param _bg_ptr StateBlock pointer to the gyrometer bias (default: nullptr).
           **/
          FrameIMU(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr, StateBlock* _ba_ptr = nullptr, StateBlock* _bg_ptr = nullptr);

          /** \brief Constructor with type, time stamp and state vector
          * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
          * \param _ts is the time stamp associated to this frame, provided in seconds
          * \param _x state vector
          **/
         FrameIMU(const FrameKeyType & _tp, const TimeStamp& _ts, Eigen::VectorXs& _x);

          /** \brief Default destructor (not recommended)
           *
           * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
           *
           **/
          virtual ~FrameIMU();

          // Frame values ------------------------------------------------

          StateBlock* getBAPtr() const;
          StateBlock* getBGPtr() const;

          void setState(const Eigen::VectorXs& _st);
          Eigen::VectorXs getState() const;
          void getState(Eigen::VectorXs& state) const;

          // Wolf tree access ---------------------------------------------------

          /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
           **/
          virtual void registerNewStateBlocks();

      private:
          /** \brief Sets the Frame status (see wolf.h for Frame status)
           **/
          void setStatus(StateStatus _st);

          Eigen::Vector3s acc_bias_at_preintegration_time_;
          Eigen::Vector3s gyro_bias_at_preintegration_time_;
  };

  // IMPLEMENTATION //

  inline StateBlock* FrameIMU::getBAPtr() const
  {
      return acc_bias_ptr_;
  }

  inline StateBlock* FrameIMU::getBGPtr() const
  {
      return gyro_bias_ptr_;
  }
} // namespace wolf

#endif
