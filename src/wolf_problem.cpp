#include "wolf_problem.h"

WolfProblem::WolfProblem() :
        NodeBase("WOLF_PROBLEM"), //
		location_(TOP),
        trajectory_ptr_(new TrajectoryBase),
		map_ptr_(new MapBase),
		hardware_ptr_(new HardwareBase)
{
    trajectory_ptr_->linkToUpperNode( this );
	map_ptr_->linkToUpperNode( this );
	hardware_ptr_->linkToUpperNode( this );
}

WolfProblem::WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr, HardwareBase* _hardware_ptr) :
        NodeBase("WOLF_PROBLEM"), //
    location_(TOP),
        trajectory_ptr_(_trajectory_ptr==nullptr ? new TrajectoryBase : _trajectory_ptr),
		map_ptr_(_map_ptr==nullptr ? new MapBase : _map_ptr),
        hardware_ptr_(_hardware_ptr==nullptr ? new HardwareBase : _hardware_ptr)
{
    trajectory_ptr_->linkToUpperNode( this );
	map_ptr_->linkToUpperNode( this );
	hardware_ptr_->linkToUpperNode( this );
}

WolfProblem::~WolfProblem()
{
    std::cout << "deleting wolf problem " << nodeId() << std::endl;
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

void WolfProblem::addStateBlockPtr(StateBlock* _state_ptr)
{
	// add the state unit to the list
	state_block_ptr_list_.push_back(_state_ptr);

	// queue for solver manager
	state_block_add_list_.push_back(_state_ptr);
}

void WolfProblem::updateStateBlockPtr(StateBlock* _state_ptr)
{
    // queue for solver manager
    state_block_update_list_.push_back(_state_ptr);
}

void WolfProblem::removeStateBlockPtr(StateBlock* _state_ptr)
{
    // add the state unit to the list
    state_block_ptr_list_.remove(_state_ptr);

    // queue for solver manager
    state_block_remove_list_.push_back(_state_ptr->getPtr());
}

void WolfProblem::addConstraintPtr(ConstraintBase* _constraint_ptr)
{
    // queue for solver manager
    constraint_add_list_.push_back(_constraint_ptr);
}

void WolfProblem::removeConstraintPtr(ConstraintBase* _constraint_ptr)
{
    // queue for solver manager
    constraint_remove_list_.remove(_constraint_ptr->nodeId());
}

void WolfProblem::clearCovariance()
{
    covariances_.clear();
}

void WolfProblem::addCovarianceBlock(StateBlock* _state1, StateBlock* _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] = _cov;
}

void WolfProblem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov_block)
{
    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose();
    else
        assert("asking for a covariance block not getted from the solver");
}

void WolfProblem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row, const int _col)
{
    //std::cout << "entire cov " << std::endl << _cov << std::endl;
    //std::cout << "_row " << _row << std::endl;
    //std::cout << "_col " << _col << std::endl;
    //if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] << std::endl;
    //else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose() << std::endl;

    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov.block(_row,_col,_state1->getSize(),_state2->getSize()) = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov.block(_row,_col,_state1->getSize(),_state2->getSize()) = covariances_[std::pair<StateBlock*, StateBlock*>(_state2, _state1)].transpose();
    else
        assert("asking for a covariance block not getted from the solver");
}

void WolfProblem::addMap(MapBase* _map_ptr)
{
    // TODO: not necessary but update map maybe..
	map_ptr_ = _map_ptr;
	map_ptr_->linkToUpperNode( this );
}

void WolfProblem::addTrajectory(TrajectoryBase* _trajectory_ptr)
{
	trajectory_ptr_ = _trajectory_ptr;
	trajectory_ptr_->linkToUpperNode( this );
}

MapBase* WolfProblem::getMapPtr()
{
	return map_ptr_;
}

TrajectoryBase* WolfProblem::getTrajectoryPtr()
{
	return trajectory_ptr_;
}

HardwareBase* WolfProblem::getHardwarePtr()
{
    return hardware_ptr_;
}

FrameBase* WolfProblem::getLastFramePtr()
{
    return trajectory_ptr_->getLastFramePtr();
}

StateBlockList* WolfProblem::getStateListPtr()
{
	return &state_block_ptr_list_;
}

void WolfProblem::print(unsigned int _ntabs, std::ostream& _ost) const
{
    printSelf(_ntabs, _ost); //one line
    printLower(_ntabs, _ost);
}

void WolfProblem::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << nodeLabel() << " " << nodeId() << " : ";
    _ost << "TOP" << std::endl;
}

std::list<StateBlock*>* WolfProblem::getStateBlockAddList()
{
    return &state_block_add_list_;
}

std::list<StateBlock*>* WolfProblem::getStateBlockUpdateList()
{
    return &state_block_update_list_;
}

std::list<WolfScalar*>* WolfProblem::getStateBlockRemoveList()
{
    return &state_block_remove_list_;
}

std::list<ConstraintBase*>* WolfProblem::getConstraintAddList()
{
    return &constraint_add_list_;
}

std::list<unsigned int>* WolfProblem::getConstraintRemoveList()
{
    return &constraint_remove_list_;
}

WolfProblem* WolfProblem::getTop()
{
	return this;
}

bool WolfProblem::isTop()
{
    return true;
}

void WolfProblem::removeDownNode(const LowerNodePtr _ptr)
{
    //
}

void WolfProblem::printLower(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << "\tLower Nodes  ==> [ ";
    _ost << map_ptr_->nodeId() << " ";
    _ost << trajectory_ptr_->nodeId() << " ";
    _ost << hardware_ptr_->nodeId() << " ]" << std::endl;
    _ntabs++;
	map_ptr_->print(_ntabs, _ost);
	trajectory_ptr_->print(_ntabs, _ost);
    hardware_ptr_->print(_ntabs, _ost);
}
