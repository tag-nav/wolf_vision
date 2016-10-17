#include "frame_imu.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

FrameIMU::FrameIMU(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _v_ptr, StateQuaternion* _o_ptr,
                   StateBlock* _ba_ptr, StateBlock* _bg_ptr) :
        FrameBase(_ts, _p_ptr, (StateBlock*)((_o_ptr)), _v_ptr), acc_bias_ptr_(_ba_ptr), gyro_bias_ptr_(_bg_ptr)
{
    setType("IMU");
//    if (acc_bias_ptr_ != nullptr)
//        acc_bias_at_preintegration_time_ = acc_bias_ptr_->getVector();
//
//    if (gyro_bias_ptr_ != nullptr)
//        gyro_bias_at_preintegration_time_ = gyro_bias_ptr_->getVector();
}

FrameIMU::FrameIMU(const FrameKeyType& _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _v_ptr,
                   StateQuaternion* _o_ptr, StateBlock* _ba_ptr, StateBlock* _bg_ptr) :
        FrameBase(_tp, _ts, _p_ptr, (StateBlock*)((_o_ptr)), _v_ptr), acc_bias_ptr_(_ba_ptr), gyro_bias_ptr_(_bg_ptr)
{
    setType("IMU");
//    if (acc_bias_ptr_ != nullptr)
//        acc_bias_at_preintegration_time_ = acc_bias_ptr_->getVector();
//
//    if (gyro_bias_ptr_ != nullptr)
//        gyro_bias_at_preintegration_time_ = gyro_bias_ptr_->getVector();
}

FrameIMU::FrameIMU(const FrameKeyType& _tp, const TimeStamp& _ts, const Eigen::VectorXs& _x) :
        FrameBase(_tp, _ts, new StateBlock(3), new StateQuaternion(), new StateBlock(3)),
        acc_bias_ptr_(new StateBlock(3)),
        gyro_bias_ptr_(new StateBlock(3))
{
    assert(_x.size() == 16 && "Wrong vector size! Must be 16.");
    setState(_x);
}


  FrameIMU::~FrameIMU()
  {
  	//std::cout << "deleting FrameIMU " << id() << std::endl;
//      is_removing_ = true;

  	// Remove Frame State Blocks
  	if (p_ptr_ != nullptr)
  	{
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(p_ptr_);
  	    delete p_ptr_;
        std::cout << "deleted  F-IMU-pos block " << p_ptr_ << std::endl;
        p_ptr_ = nullptr;
  	}
      if (v_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(v_ptr_);
          delete v_ptr_;
          std::cout << "deleted  F-IMU-vel block " << v_ptr_ << std::endl;
          v_ptr_ = nullptr;
      }
      if (o_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(o_ptr_);
          delete o_ptr_;
          std::cout << "deleted  F-IMU-ori block " << o_ptr_ << std::endl;
          o_ptr_ = nullptr;
      }
      if (acc_bias_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(acc_bias_ptr_);
          delete acc_bias_ptr_;
          std::cout << "deleted  F-IMU-ab block " << acc_bias_ptr_ << std::endl;
          acc_bias_ptr_ = nullptr;
      }
      if (gyro_bias_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(gyro_bias_ptr_);
          delete gyro_bias_ptr_;
          std::cout << "deleted  F-IMU-wb block " << gyro_bias_ptr_ << std::endl;
          gyro_bias_ptr_ = nullptr;
      }


      //std::cout << "states deleted" << std::endl;


//      while (!getConstrainedByListPtr()->empty())
//      {
//          //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
//          getConstrainedByListPtr()->front()->remove();
//          //std::cout << "deleted " << std::endl;
//      }
      //std::cout << "constraints deleted" << std::endl;
  }

  void FrameIMU::registerNewStateBlocks()
  {
      if (getProblem() != nullptr)
      {
          if (p_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(p_ptr_);

          if (v_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(v_ptr_);

          if (o_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(o_ptr_);

          if (acc_bias_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(acc_bias_ptr_);

          if (gyro_bias_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(gyro_bias_ptr_);
      }
  }

  void FrameIMU::setState(const Eigen::VectorXs& _st) // Order: PVQ
  {

      assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                            (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                            (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                            (acc_bias_ptr_==nullptr ? 0 : acc_bias_ptr_->getSize())  +
                            (gyro_bias_ptr_==nullptr ? 0 : gyro_bias_ptr_->getSize())) &&
                            "In FrameBase::setState wrong state size, should be 16!");

      unsigned int index = 0;
      if (p_ptr_!=nullptr)
      {
          p_ptr_->setVector(_st.head(p_ptr_->getSize()));
          index += p_ptr_->getSize();
          std::cout << "set F-pos" << std::endl;
      }
      if (v_ptr_!=nullptr)
      {
          v_ptr_->setVector(_st.segment(index, v_ptr_->getSize()));
          index += v_ptr_->getSize();
          std::cout << "set F-vel" << std::endl;
      }
      if (o_ptr_!=nullptr)
      {
          o_ptr_->setVector(_st.segment(index, o_ptr_->getSize()));
          index += o_ptr_->getSize();
          std::cout << "set F-ori" << std::endl;
      }
      if (acc_bias_ptr_!=nullptr)
      {
          acc_bias_ptr_->setVector(_st.segment(index, acc_bias_ptr_->getSize()));
          index += acc_bias_ptr_->getSize();
          std::cout << "set F-ab" << std::endl;
      }
      if (gyro_bias_ptr_!=nullptr)
      {
          gyro_bias_ptr_->setVector(_st.segment(index, gyro_bias_ptr_->getSize()));
          //   index += bg_ptr_->getSize();
          std::cout << "set F-wb" << std::endl;
     }
  }

  Eigen::VectorXs FrameIMU::getState() const
  {
      Eigen::VectorXs state((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                            (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                            (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                            (acc_bias_ptr_==nullptr ? 0 : acc_bias_ptr_->getSize())  +
                            (gyro_bias_ptr_==nullptr ? 0 : gyro_bias_ptr_->getSize()));

      getState(state);

      return state;
  }

  void FrameIMU::getState(Eigen::VectorXs& state) const // Order: PVQBB
  {
      assert(state.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                              (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                              (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                              (acc_bias_ptr_==nullptr ? 0 : acc_bias_ptr_->getSize())  +
                              (gyro_bias_ptr_==nullptr ? 0 : gyro_bias_ptr_->getSize())));

      unsigned int index = 0;
      if (p_ptr_!=nullptr)
      {
          state.head(p_ptr_->getSize()) = p_ptr_->getVector();
          index += p_ptr_->getSize();
      }
      if (v_ptr_!=nullptr)
      {
          state.segment(index, v_ptr_->getSize()) = v_ptr_->getVector();
          index += v_ptr_->getSize();
      }
      if (o_ptr_!=nullptr)
      {
          state.segment(index, o_ptr_->getSize()) = o_ptr_->getVector();
          index += o_ptr_->getSize();
      }
      if (acc_bias_ptr_!=nullptr)
      {
          state.segment(index, acc_bias_ptr_->getSize()) = acc_bias_ptr_->getVector();
          index += acc_bias_ptr_->getSize();
      }
      if (gyro_bias_ptr_!=nullptr)
      {
          state.segment(index, gyro_bias_ptr_->getSize()) = gyro_bias_ptr_->getVector();
        //  index += bg_ptr_->getSize();
      }
  }

  void FrameIMU::setStatus(StateStatus _st)
  {
      // TODO: Separate the three fixes and unfixes to the wolfproblem lists
      status_ = _st;
      // State Blocks
      if (status_ == ST_FIXED)
      {
          if (p_ptr_ != nullptr)
          {
              p_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(p_ptr_);
          }
          if (v_ptr_ != nullptr)
          {
              v_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(v_ptr_);
          }
          if (o_ptr_ != nullptr)
          {
              o_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(o_ptr_);
          }
          if (acc_bias_ptr_ != nullptr)
          {
              acc_bias_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(acc_bias_ptr_);
          }
          if (gyro_bias_ptr_ != nullptr)
          {
              gyro_bias_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(gyro_bias_ptr_);
          }
      }
      else if (status_ == ST_ESTIMATED)
      {
          if (p_ptr_ != nullptr)
          {
              p_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(p_ptr_);
          }
          if (v_ptr_ != nullptr)
          {
              v_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(v_ptr_);
          }
          if (o_ptr_ != nullptr)
          {
              o_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(o_ptr_);
          }
          if (acc_bias_ptr_ != nullptr)
          {
              acc_bias_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(acc_bias_ptr_);
          }
          if (gyro_bias_ptr_ != nullptr)
          {
              gyro_bias_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(gyro_bias_ptr_);
          }
      }
  }
}
