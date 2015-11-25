#include "wolf_problem.h"

WolfProblem::WolfProblem(unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
        covariance_(_size,_size),
		location_(TOP),
        trajectory_ptr_(new TrajectoryBase),
		map_ptr_(new MapBase),
		hardware_ptr_(new HardwareBase)
{
    trajectory_ptr_->linkToUpperNode( this );
	map_ptr_->linkToUpperNode( this );
	hardware_ptr_->linkToUpperNode( this );
}

WolfProblem::WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr, HardwareBase* _hardware_ptr, unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
		covariance_(_size,_size),
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
	delete trajectory_ptr_;
    delete map_ptr_;
    delete hardware_ptr_;
}

void WolfProblem::addStateBlockPtr(StateBlock* _state_ptr)
{
	// add the state unit to the list
	state_block_ptr_list_.push_back(_state_ptr);
	state_idx_map_[_state_ptr] = covariance_.rows();

	// Resize Covariance
	covariance_.conservativeResize(covariance_.rows() + _state_ptr->getSize(), covariance_.cols() + _state_ptr->getSize());
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
    state_idx_map_.erase(_state_ptr);

    // Resize Covariance
    covariance_.conservativeResize(covariance_.rows() - _state_ptr->getSize(), covariance_.cols() - _state_ptr->getSize());
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
//    assert(_state1 != nullptr);
//    assert(state_idx_map_.find(_state1) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state1) + _state1->getSize() <= (unsigned int) covariance_.rows());
//    assert(_state2 != nullptr);
//    assert(state_idx_map_.find(_state2) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state2) + _state2->getSize() <= (unsigned int) covariance_.rows());
//
//    // Guarantee that we are updating the top triangular matrix (in cross covariance case)
//    bool flip = state_idx_map_.at(_state1) > state_idx_map_.at(_state2);
//    StateBlock* stateA = (flip ? _state2 : _state1);
//    StateBlock* stateB = (flip ? _state1 : _state2);
//    unsigned int row = state_idx_map_.at(stateA);
//    unsigned int col = state_idx_map_.at(stateB);
//    unsigned int block_rows = stateA->getSize();
//    unsigned int block_cols = stateB->getSize();
//
//    assert( block_rows == (flip ? _cov.cols() : _cov.rows()) && block_cols == (flip ? _cov.rows() : _cov.cols()) && "Bad covariance size in WolfProblem::addCovarianceBlock");
//
//    // STORE COVARIANCE
//    for (unsigned int i = 0; i < block_rows; i++)
//       for (unsigned int j = 0; j < block_cols; j++)
//           covariance_.coeffRef(i+row,j+col) = (flip ? _cov(j,i) : _cov(i,j));

    assert(_state1->getSize() == (unsigned int) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)] = _cov;
}

void WolfProblem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov_block)
{
//    assert(_state1 != nullptr);
//    assert(state_idx_map_.find(_state1) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state1) + _state1->getSize() <= (unsigned int) covariance_.rows());
//    assert(_state2 != nullptr);
//    assert(state_idx_map_.find(_state2) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state2) + _state2->getSize() <= (unsigned int) covariance_.rows());
//
//    // Guarantee that we are updating the top triangular matrix (in cross covariance case)
//    bool flip = state_idx_map_.at(_state1) > state_idx_map_.at(_state2);
//    StateBlock* stateA = (flip ? _state2 : _state1);
//    StateBlock* stateB = (flip ? _state1 : _state2);
//    unsigned int row = state_idx_map_.at(stateA);
//    unsigned int col = state_idx_map_.at(stateB);
//    unsigned int block_rows = stateA->getSize();
//    unsigned int block_cols = stateB->getSize();
//
//    assert(_cov_block.rows() == (flip ? block_cols : block_rows) && _cov_block.cols() == (flip ? block_rows : block_cols) && "Bad _cov_block matrix sizes");
//
//    _cov_block = (flip ? Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)) : Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)).transpose() );

    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov_block = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)].transpose();
    else
        assert("asking for a covariance block not getted from the solver");
}

void WolfProblem::getCovarianceBlock(StateBlock* _state1, StateBlock* _state2, Eigen::MatrixXs& _cov, const int _row, const int _col)
{
//    assert(_state1 != nullptr);
//    assert(state_idx_map_.find(_state1) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state1) + _state1->getSize() <= (unsigned int) covariance_.rows());
//    assert(_state2 != nullptr);
//    assert(state_idx_map_.find(_state2) != state_idx_map_.end());
//    assert(state_idx_map_.at(_state2) + _state2->getSize() <= (unsigned int) covariance_.rows());
//
//    // Guarantee that we are updating the top triangular matrix (in cross covariance case)
//    bool flip = state_idx_map_.at(_state1) > state_idx_map_.at(_state2);
//    StateBlock* stateA = (flip ? _state2 : _state1);
//    StateBlock* stateB = (flip ? _state1 : _state2);
//    unsigned int row = state_idx_map_.at(stateA);
//    unsigned int col = state_idx_map_.at(stateB);
//    unsigned int block_rows = stateA->getSize();
//    unsigned int block_cols = stateB->getSize();
//
////    std::cout << "flip " << flip << std::endl;
////    std::cout << "_row " << _row << std::endl;
////    std::cout << "_cov.rows() " << _cov.rows() << std::endl;
////    std::cout << "block_rows " << block_rows << std::endl;
////    std::cout << "_col " << _col << std::endl;
////    std::cout << "_cov.cols() " << _cov.cols() << std::endl;
////    std::cout << "block_cols " << block_cols << std::endl;
//
//    assert(_cov.rows() - _row >= (flip ? block_cols : block_rows) && _cov.cols() - _col >= (flip ? block_rows : block_cols) && "Bad _cov_block matrix sizes");
//
//    if (!flip)
//        _cov.block(_row,_col,block_rows,block_cols) = Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols));
//    else
//        _cov.block(_row,_col,block_cols,block_rows) = Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)).transpose();

    if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state1, _state2)) != covariances_.end())
        _cov.block(_row,_col,_state1->getSize(),_state2->getSize()) = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlock*, StateBlock*>(_state2, _state1)) != covariances_.end())
        _cov.block(_row,_col,_state2->getSize(),_state1->getSize()) = covariances_[std::pair<StateBlock*, StateBlock*>(_state1, _state2)].transpose();
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

void WolfProblem::printLower(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << "\tLower Nodes  ==> [ ";
    _ost << map_ptr_->nodeId() << " ";
    _ost << trajectory_ptr_->nodeId() << " ]" << std::endl;
    _ntabs++;
	map_ptr_->print(_ntabs, _ost);
	trajectory_ptr_->print(_ntabs, _ost);
}
