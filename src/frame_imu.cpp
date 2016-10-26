#include "frame_imu.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

FrameIMU::FrameIMU(const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _v_ptr, StateQuaternionPtr _q_ptr,
                   StateBlockPtr _ba_ptr, StateBlockPtr _bg_ptr) :
        FrameBase(_ts, _p_ptr, (StateBlockPtr)((_q_ptr)), _v_ptr)//,
{
    setStateBlockPtr(3, std::make_shared<StateBlock>(3)); // acc bias
    setStateBlockPtr(4, std::make_shared<StateBlock>(3)); // gyro bias
    setType("IMU");
}

FrameIMU::FrameIMU(const FrameKeyType& _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _v_ptr,
                   StateQuaternionPtr _q_ptr, StateBlockPtr _ba_ptr, StateBlockPtr _bg_ptr) :
        FrameBase(_tp, _ts, _p_ptr, (StateBlockPtr)((_q_ptr)), _v_ptr)
{
    setStateBlockPtr(3, std::make_shared<StateBlock>(3)); // acc bias
    setStateBlockPtr(4, std::make_shared<StateBlock>(3)); // gyro bias
    setType("IMU");
}

FrameIMU::FrameIMU(const FrameKeyType& _tp, const TimeStamp& _ts, const Eigen::VectorXs& _x) :
        FrameBase(_tp, _ts, std::make_shared<StateBlock>(3), std::make_shared<StateQuaternion>(), std::make_shared<StateBlock>(3))
{
    setStateBlockPtr(3, std::make_shared<StateBlock>(3)); // acc bias
    setStateBlockPtr(4, std::make_shared<StateBlock>(3)); // gyro bias
    assert(_x.size() == 16 && "Wrong vector size! Must be 16.");
    setState(_x);
    setType("IMU");
}


  FrameIMU::~FrameIMU()
  {
      std::cout << "destructed   -F-IMU" << id() << std::endl;
  }

  void FrameIMU::setState(const Eigen::VectorXs& _st) // Order: PVQ
  {

      assert(_st.size() == ((getPPtr()==nullptr ? 0 : getPPtr()->getSize())    +
                            (getVPtr()==nullptr ? 0 : getVPtr()->getSize())    +
                            (getOPtr()==nullptr ? 0 : getOPtr()->getSize())    +
                            (getAccBiasPtr()==nullptr ? 0 : getAccBiasPtr()->getSize())  +
                            (getGyroBiasPtr()==nullptr ? 0 : getGyroBiasPtr()->getSize())) &&
                            "In FrameBase::setState wrong state size, should be 16!");

      unsigned int index = 0;
      if (getPPtr()!=nullptr)
      {
          getPPtr()->setVector(_st.head(getPPtr()->getSize()));
          index += getPPtr()->getSize();
//          std::cout << "set F-pos" << std::endl;
      }
      if (getVPtr()!=nullptr)
      {
          getVPtr()->setVector(_st.segment(index, getVPtr()->getSize()));
          index += getVPtr()->getSize();
//          std::cout << "set F-vel" << std::endl;
      }
      if (getOPtr()!=nullptr)
      {
          getOPtr()->setVector(_st.segment(index, getOPtr()->getSize()));
          index += getOPtr()->getSize();
//          std::cout << "set F-ori" << std::endl;
      }
      if (getAccBiasPtr()!=nullptr)
      {
          getAccBiasPtr()->setVector(_st.segment(index, getAccBiasPtr()->getSize()));
          index += getAccBiasPtr()->getSize();
//          std::cout << "set F-ab" << std::endl;
      }
      if (getGyroBiasPtr()!=nullptr)
      {
          getGyroBiasPtr()->setVector(_st.segment(index, getGyroBiasPtr()->getSize()));
          //   index += bg_ptr_->getSize();
//          std::cout << "set F-wb" << std::endl;
     }
  }

  Eigen::VectorXs FrameIMU::getState() const
  {
      Eigen::VectorXs state((getPPtr()==nullptr ? 0 : getPPtr()->getSize())    +
                            (getVPtr()==nullptr ? 0 : getVPtr()->getSize())    +
                            (getOPtr()==nullptr ? 0 : getOPtr()->getSize())    +
                            (getAccBiasPtr()==nullptr ? 0 : getAccBiasPtr()->getSize())  +
                            (getGyroBiasPtr()==nullptr ? 0 : getGyroBiasPtr()->getSize()));

      getState(state);

      return state;
  }

  void FrameIMU::getState(Eigen::VectorXs& state) const // Order: PVQBB
  {
      assert(state.size() == ((getPPtr()==nullptr ? 0 : getPPtr()->getSize())    +
                              (getVPtr()==nullptr ? 0 : getVPtr()->getSize())    +
                              (getOPtr()==nullptr ? 0 : getOPtr()->getSize())    +
                              (getAccBiasPtr()==nullptr ? 0 : getAccBiasPtr()->getSize())  +
                              (getGyroBiasPtr()==nullptr ? 0 : getGyroBiasPtr()->getSize())));

      unsigned int index = 0;
      if (getPPtr()!=nullptr)
      {
          state.head(getPPtr()->getSize()) = getPPtr()->getVector();
          index += getPPtr()->getSize();
      }
      if (getVPtr()!=nullptr)
      {
          state.segment(index, getVPtr()->getSize()) = getVPtr()->getVector();
          index += getVPtr()->getSize();
      }
      if (getOPtr()!=nullptr)
      {
          state.segment(index, getOPtr()->getSize()) = getOPtr()->getVector();
          index += getOPtr()->getSize();
      }
      if (getAccBiasPtr()!=nullptr)
      {
          state.segment(index, getAccBiasPtr()->getSize()) = getAccBiasPtr()->getVector();
          index += getAccBiasPtr()->getSize();
      }
      if (getGyroBiasPtr()!=nullptr)
      {
          state.segment(index, getGyroBiasPtr()->getSize()) = getGyroBiasPtr()->getVector();
        //  index += bg_ptr_->getSize();
      }
  }

  void FrameIMU::setStatus(StateStatus _st)
  {
      // TODO: Separate the three fixes and unfixes to the wolfproblem lists
      // TODO: See what we want to do with globally fixing state blocks
      status_ = _st;
      // State Blocks
      if (status_ == ST_FIXED)
      {
          if (getPPtr() != nullptr)
          {
              getPPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getPPtr());
          }
          if (getVPtr() != nullptr)
          {
              getVPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getVPtr());
          }
          if (getOPtr() != nullptr)
          {
              getOPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getOPtr());
          }
          if (getAccBiasPtr() != nullptr)
          {
              getAccBiasPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getAccBiasPtr());
          }
          if (getGyroBiasPtr() != nullptr)
          {
              getGyroBiasPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getGyroBiasPtr());
          }
      }
      else if (status_ == ST_ESTIMATED)
      {
          if (getPPtr() != nullptr)
          {
              getPPtr()->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getPPtr());
          }
          if (getVPtr() != nullptr)
          {
              getVPtr()->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getVPtr());
          }
          if (getOPtr() != nullptr)
          {
              getOPtr()->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getOPtr());
          }
          if (getAccBiasPtr() != nullptr)
          {
              getAccBiasPtr()->unfix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getAccBiasPtr());
          }
          if (getGyroBiasPtr() != nullptr)
          {
              getGyroBiasPtr()->fix();
              if (getProblem() != nullptr)
                  getProblem()->updateStateBlockPtr(getGyroBiasPtr());
          }
      }
  }



FrameBasePtr FrameIMU::create(const FrameKeyType & _tp,
                              const TimeStamp& _ts,
                              const Eigen::VectorXs& _x)
{

    assert(_x.size() == 16 && "Wrond state vector size. Should be 16 for an IMU with biases!");

    StateBlockPtr       p_ptr = std::make_shared<StateBlock>      (_x.segment<3>( 0  ));
    StateQuaternionPtr  q_ptr = std::make_shared<StateQuaternion> (_x.segment<4>( 3  ));
    StateBlockPtr       v_ptr = std::make_shared<StateBlock>      (_x.segment<3>( 7  ));
    StateBlockPtr       a_ptr = std::make_shared<StateBlock>      (_x.segment<3>( 10 ));
    StateBlockPtr       w_ptr = std::make_shared<StateBlock>      (_x.segment<3>( 13 ));

    return std::make_shared<FrameIMU>(_tp, _ts, p_ptr, v_ptr, q_ptr, a_ptr, w_ptr);
}

} // namespace wolf

#include "factory.h"
namespace wolf
{
WOLF_REGISTER_FRAME("IMU", FrameIMU)
} // namespace wolf
