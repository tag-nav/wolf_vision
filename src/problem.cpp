#include "problem.h"
#include "constraint_base.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "hardware_base.h"
#include "trajectory_base.h"
#include "map_base.h"
#include "processor_motion.h"

namespace wolf
{

Problem::Problem(FrameStructure _frame_structure) :
        NodeBase("WOLF_PROBLEM"), //
        location_(TOP), trajectory_ptr_(new TrajectoryBase(_frame_structure)), map_ptr_(new MapBase), hardware_ptr_(
                new HardwareBase), processor_motion_ptr_(nullptr)
{
    trajectory_ptr_->linkToUpperNode(this);
    map_ptr_->linkToUpperNode(this);
    hardware_ptr_->linkToUpperNode(this);
}

Problem::~Problem()
{
    //std::cout << "deleting wolf problem " << nodeId() << std::endl;
    state_block_add_list_.clear();
    covariances_.clear();
    state_block_update_list_.clear();
    state_block_remove_list_.clear();
    constraint_add_list_.clear();
    constraint_remove_list_.clear();

    trajectory_ptr_->destruct();
    map_ptr_->destruct();
    hardware_ptr_->destruct();
}

void Problem::destruct()
{
    delete this;
}

void Problem::addSensor(SensorBase* _sen_ptr)
{
    getHardwarePtr()->addSensor(_sen_ptr);
}

void Problem::setProcessorMotion(ProcessorMotion* _processor_motion_ptr)
{
    processor_motion_ptr_ = _processor_motion_ptr;
}

FrameBase* Problem::createFrame(FrameType _frame_type, const TimeStamp& _time_stamp)
{
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
        {
            trajectory_ptr_->addFrame(new FrameBase(_frame_type, _time_stamp, new StateBlock(2), new StateBlock(1)));
            break;
        }
        case FRM_PO_3D:
        {
            trajectory_ptr_->addFrame(new FrameBase(_frame_type, _time_stamp, new StateBlock(3), new StateQuaternion));
            break;
        }
        case FRM_POV_3D:
        {
            trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(3), new StateQuaternion, new StateBlock(3)));
            break;
        }
        default:
        {
            assert("Unknown frame structure. Add appropriate frame structure to the switch statement.");
        }
    }
    return trajectory_ptr_->getLastFramePtr();
}

FrameBase* Problem::createFrame(FrameType _frame_type, const Eigen::VectorXs& _frame_state,
                                    const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame..." << std::endl;

    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch (trajectory_ptr_->getFrameStructure())
    {
        case FRM_PO_2D:
        {
            assert(_frame_state.size() == 3 && "Wrong state vector size");

            trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(2)),
                                  new StateBlock(_frame_state.tail(1))));
            break;
        }
        case FRM_PO_3D:
        {
            assert(_frame_state.size() == 7 && "Wrong state vector size");

            trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(3)),
                                  new StateQuaternion(_frame_state.tail(4))));
            break;
        }
        case FRM_POV_3D:
        {
            assert(_frame_state.size() == 10 && "Wrong state vector size");

            trajectory_ptr_->addFrame(
                    new FrameBase(_frame_type, _time_stamp, new StateBlock(_frame_state.head(3)),
                                  new StateQuaternion(_frame_state.segment<3>(4)),
                                  new StateBlock(_frame_state.tail(3))));
            break;
        }
        default:
        {
            assert("Unknown frame structure. Add appropriate frame structure to the switch statement.");
        }
    }
    //std::cout << "new frame created" << std::endl;
    return trajectory_ptr_->getLastFramePtr();
}

void Problem::getCurrentState(Eigen::VectorXs& state)
{
    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->state(state);
    else
        throw std::runtime_error("WolfProblem::getCurrentState: processor motion not set!");
}

Eigen::VectorXs Problem::getCurrentState()
{
    if (processor_motion_ptr_ != nullptr)
        return processor_motion_ptr_->state();
    else
        throw std::runtime_error("WolfProblem::getCurrentState: processor motion not set!");
}

void Problem::getStateAtTimeStamp(const TimeStamp& _ts, Eigen::VectorXs& state)
{
    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->state(_ts, state);
    else
        throw std::runtime_error("WolfProblem::getCurrentState: processor motion not set!");
}

Eigen::VectorXs Problem::getStateAtTimeStamp(const TimeStamp& _ts)
{
    if (processor_motion_ptr_ != nullptr)
        return processor_motion_ptr_->state(_ts);
    else
        throw std::runtime_error("WolfProblem::getCurrentState: processor motion not set!");
}

bool Problem::permitKeyFrame(ProcessorBase* _processor_ptr)
{
    return true;
}

void Problem::addLandmark(LandmarkBase* _lmk_ptr)
{
    getMapPtr()->addLandmark(_lmk_ptr);
}

void Problem::addLandmarkList(LandmarkBaseList _lmk_list)
{
    getMapPtr()->addLandmarkList(_lmk_list);
}

void Problem::addStateBlockPtr(StateBlock* _state_ptr)
{
    // add the state unit to the list
    state_block_ptr_list_.push_back(_state_ptr);
    // queue for solver manager
    state_block_add_list_.push_back(_state_ptr);
}

void Problem::updateStateBlockPtr(StateBlock* _state_ptr)
{
    // queue for solver manager
    state_block_update_list_.push_back(_state_ptr);
}

void Problem::removeStateBlockPtr(StateBlock* _state_ptr)
{
    // add the state unit to the list
    state_block_ptr_list_.remove(_state_ptr);
    // queue for solver manager
    state_block_remove_list_.push_back(_state_ptr->getPtr());
}

void Problem::addConstraintPtr(ConstraintBase* _constraint_ptr)
{
    // queue for solver manager
    constraint_add_list_.push_back(_constraint_ptr);
}

void Problem::removeConstraintPtr(ConstraintBase* _constraint_ptr)
{
    // queue for solver manager
    constraint_remove_list_.push_back(_constraint_ptr->nodeId());
}

void Problem::clearCovariance()
{
    covariances_.clear();
}

void Problem::addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] = _cov;
}

bool Problem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov_block)
{
    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose();
    else
        return false;

    return true;
}

bool Problem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row,
                                     const int _col)
{
    //std::cout << "entire cov " << std::endl << _cov << std::endl;
    //std::cout << "_row " << _row << std::endl;
    //std::cout << "_col " << _col << std::endl;
    //if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] << std::endl;
    //else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose() << std::endl;

    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose();
    else
        return false;

    return true;
}

void Problem::addMap(MapBase* _map_ptr)
{
    // TODO: not necessary but update map maybe..
    map_ptr_ = _map_ptr;
    map_ptr_->linkToUpperNode(this);
}

void Problem::addTrajectory(TrajectoryBase* _trajectory_ptr)
{
    trajectory_ptr_ = _trajectory_ptr;
    trajectory_ptr_->linkToUpperNode(this);
}

MapBase* Problem::getMapPtr()
{
    return map_ptr_;
}

TrajectoryBase* Problem::getTrajectoryPtr()
{
    return trajectory_ptr_;
}

HardwareBase* Problem::getHardwarePtr()
{
    return hardware_ptr_;
}

FrameBase* Problem::getLastFramePtr()
{
    return trajectory_ptr_->getLastFramePtr();
}

StateBlockList* Problem::getStateListPtr()
{
    return &state_block_ptr_list_;
}

std::list<StateBlock*>* Problem::getStateBlockAddList()
{
    return &state_block_add_list_;
}

std::list<StateBlock*>* Problem::getStateBlockUpdateList()
{
    return &state_block_update_list_;
}

} // namespace wolf
