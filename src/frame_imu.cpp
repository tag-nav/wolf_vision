#include "frame_imu.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"

namespace wolf {

  FrameIMU::FrameIMU(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr, StateBlock* _ba_ptr, StateBlock* _bg_ptr) :
              FrameBase(_ts, _p_ptr, _o_ptr, _v_ptr),
              ba_ptr_(_ba_ptr),
              bg_ptr_(_bg_ptr)
  {
      setType("IMU");
  }

  FrameIMU::FrameIMU(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr, StateBlock* _ba_ptr, StateBlock* _bg_ptr) :
            FrameBase(_tp, _ts, _p_ptr, _o_ptr, _v_ptr),
            ba_ptr_(_ba_ptr),
            bg_ptr_(_bg_ptr)
  {
      setType("IMU");
  }

  FrameIMU::~FrameIMU()
  {
  	//std::cout << "deleting FrameIMU " << id() << std::endl;
      is_deleting_ = true;

  	// Remove Frame State Blocks
  	if (p_ptr_ != nullptr)
  	{
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(p_ptr_);
  	    delete p_ptr_;
  	}
      if (o_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(o_ptr_);
          delete o_ptr_;
      }
      if (v_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(v_ptr_);
          delete v_ptr_;
      }
      if (ba_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(ba_ptr_);
          delete ba_ptr_;
      }
      if (bg_ptr_ != nullptr)
      {
          if (getProblem() != nullptr && type_id_ == KEY_FRAME)
              getProblem()->removeStateBlockPtr(bg_ptr_);
          delete bg_ptr_;
      }


      //std::cout << "states deleted" << std::endl;


      while (!getConstrainedByListPtr()->empty())
      {
          //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
          getConstrainedByListPtr()->front()->destruct();
          //std::cout << "deleted " << std::endl;
      }
      //std::cout << "constraints deleted" << std::endl;
  }

  void FrameIMU::registerNewStateBlocks()
  {
      if (getProblem() != nullptr)
      {
          if (p_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(p_ptr_);

          if (o_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(o_ptr_);

          if (v_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(v_ptr_);

          if (ba_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(ba_ptr_);

          if (bg_ptr_ != nullptr)
              getProblem()->addStateBlockPtr(bg_ptr_);
      }
  }

  void FrameIMU::setState(const Eigen::VectorXs& _st)
  {

      assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                            (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                            (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                            (ba_ptr_==nullptr ? 0 : ba_ptr_->getSize())  +
                            (bg_ptr_==nullptr ? 0 : bg_ptr_->getSize())) &&
                            "In FrameBase::setState wrong state size");

      unsigned int index = 0;
      if (p_ptr_!=nullptr)
      {
          p_ptr_->setVector(_st.head(p_ptr_->getSize()));
          index += p_ptr_->getSize();
      }
      if (o_ptr_!=nullptr)
      {
          o_ptr_->setVector(_st.segment(index, o_ptr_->getSize()));
          index += o_ptr_->getSize();
      }
      if (v_ptr_!=nullptr)
      {
          v_ptr_->setVector(_st.segment(index, v_ptr_->getSize()));
          index += v_ptr_->getSize();
      }
      if (ba_ptr_!=nullptr)
      {
          ba_ptr_->setVector(_st.segment(index, ba_ptr_->getSize()));
          index += ba_ptr_->getSize();
      }
      if (bg_ptr_!=nullptr)
      {
          bg_ptr_->setVector(_st.segment(index, bg_ptr_->getSize()));
          //   index += bg_ptr_->getSize();
      }
  }

  Eigen::VectorXs FrameIMU::getState() const
  {
      Eigen::VectorXs state((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                            (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                            (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                            (ba_ptr_==nullptr ? 0 : ba_ptr_->getSize())  +
                            (bg_ptr_==nullptr ? 0 : bg_ptr_->getSize()));

      getState(state);

      return state;
  }

  void FrameIMU::getState(Eigen::VectorXs& state) const
  {
      assert(state.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())    +
                              (o_ptr_==nullptr ? 0 : o_ptr_->getSize())    +
                              (v_ptr_==nullptr ? 0 : v_ptr_->getSize())    +
                              (ba_ptr_==nullptr ? 0 : ba_ptr_->getSize())  +
                              (bg_ptr_==nullptr ? 0 : bg_ptr_->getSize())));

      unsigned int index = 0;
      if (p_ptr_!=nullptr)
      {
          state.head(p_ptr_->getSize()) = p_ptr_->getVector();
          index += p_ptr_->getSize();
      }
      if (o_ptr_!=nullptr)
      {
          state.segment(index, o_ptr_->getSize()) = o_ptr_->getVector();
          index += o_ptr_->getSize();
      }
      if (v_ptr_!=nullptr)
      {
          state.segment(index, v_ptr_->getSize()) = v_ptr_->getVector();
          index += v_ptr_->getSize();
      }
      if (ba_ptr_!=nullptr)
      {
          state.segment(index, ba_ptr_->getSize()) = ba_ptr_->getVector();
          index += ba_ptr_->getSize();
      }
      if (bg_ptr_!=nullptr)
      {
          state.segment(index, bg_ptr_->getSize()) = bg_ptr_->getVector();
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
          if (o_ptr_ != nullptr)
          {
              o_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(o_ptr_);
          }
          if (v_ptr_ != nullptr)
          {
              v_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(v_ptr_);
          }
          if (ba_ptr_ != nullptr)
          {
              ba_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(ba_ptr_);
          }
          if (bg_ptr_ != nullptr)
          {
              bg_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(bg_ptr_);
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
          if (o_ptr_ != nullptr)
          {
              o_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(o_ptr_);
          }
          if (v_ptr_ != nullptr)
          {
              v_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(v_ptr_);
          }
          if (ba_ptr_ != nullptr)
          {
              ba_ptr_->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(ba_ptr_);
          }
          if (bg_ptr_ != nullptr)
          {
              bg_ptr_->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(bg_ptr_);
          }
      }
  }
}
