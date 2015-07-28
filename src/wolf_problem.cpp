#include "wolf_problem.h"

RadarOdom+::WolfProblem(unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
        state_(_size),
        covariance_(_size,_size),
		state_idx_last_(0),
        location_(TOP),
		map_ptr_(new MapBase),
		trajectory_ptr_(new TrajectoryBase),
        reallocated_(false)
{
//	map_ptr_ = new MapBase;
//	trajectory_ptr_ = new TrajectoryBase;
    map_ptr_->linkToUpperNode( this );
	trajectory_ptr_->linkToUpperNode( this );
}

RadarOdom+::WolfProblem(TrajectoryBase* _trajectory_ptr, MapBase* _map_ptr, unsigned int _size) :
        NodeBase("WOLF_PROBLEM"), //
		state_(_size),
        covariance_(_size,_size),
		state_idx_last_(0),
        location_(TOP),
		map_ptr_(_map_ptr==nullptr ? new MapBase : _map_ptr),
		trajectory_ptr_(_trajectory_ptr==nullptr ? new TrajectoryBase : _trajectory_ptr),
        reallocated_(false)
{
	map_ptr_->linkToUpperNode( this );
	trajectory_ptr_->linkToUpperNode( this );
}

RadarOdom+::~RadarOdom+()
{
	delete trajectory_ptr_;
	delete map_ptr_;
}

bool RadarOdom+::addState(StateBase* _new_state_ptr, const Eigen::VectorXs& _new_state_values)
{
	// Check if resize should be done
	if (state_idx_last_+_new_state_ptr->getStateSize() > state_.size())
	{
		std::cout << "Resizing state and updating asl state units pointers..." << std::endl;
		std::cout << "\nState size: " << state_.size() << " last idx: " << state_idx_last_ << " last idx + new state size: " << state_idx_last_+_new_state_ptr->getStateSize() << std::endl;
        WolfScalar* old_first_pointer = state_.data();
		state_.conservativeResize(state_.size()*2);
		covariance_.conservativeResize(state_.size()*2,state_.size()*2);
		for (auto state_units_it = state_list_.begin(); state_units_it != state_list_.end(); state_units_it++)
		{
	        //std::cout << "state unit: " << (*state_units_it)->nodeId() << std::endl;
		    (*state_units_it)->setPtr(state_.data() + ( (*state_units_it)->getPtr() - old_first_pointer) );
		}
		std::cout << "----------------------------- difference of location: " << old_first_pointer - state_.data() << std::endl;
		_new_state_ptr->setPtr(state_.data()+state_idx_last_);
		std::cout << "New state size: " << state_.size() << " last idx: " << state_idx_last_ << std::endl;
		reallocated_ = true;
	}
	//std::cout << "\nnew state unit: " << _new_state_values.transpose() << std::endl;
	//std::cout << "\nPrev state: " << state_.segment(0,state_idx_last_).transpose() << std::endl;

	// copy the values of the new state
	assert(_new_state_values.size() == _new_state_ptr->getStateSize() && "Different state unit and vector sizes");
	state_.segment(state_idx_last_,_new_state_ptr->getStateSize()) = _new_state_values;

	// add the state unit to the list
	state_list_.push_back(_new_state_ptr);

	// update the last state index
	state_idx_last_ += _new_state_ptr->getStateSize();

	//std::cout << "\nPost state: " << state_.segment(0,state_idx_last_).transpose() << std::endl;
	return reallocated_;
}

void RadarOdom+::addCovarianceBlock(StateBase* _state1, StateBase* _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1 != nullptr);
    assert(_state1->getPtr() != nullptr);
    assert(_state1->getPtr() < state_.data() + state_idx_last_);
    assert(_state1->getPtr() > state_.data());
    assert(_state2 != nullptr);
    assert(_state2->getPtr() != nullptr);
    assert(_state2->getPtr() < state_.data() + state_idx_last_);
    assert(_state2->getPtr() > state_.data());

    // Guarantee that we are updating the top triangular matrix (in cross covariance case)
    bool flip = _state1->getPtr() > _state2->getPtr();
    StateBase* stateA = (flip ? _state2 : _state1);
    StateBase* stateB = (flip ? _state1 : _state2);
    unsigned int row = (stateA->getPtr() - state_.data());
    unsigned int col = (stateB->getPtr() - state_.data());
    unsigned int block_rows = stateA->getStateSize();
    unsigned int block_cols = stateB->getStateSize();

    assert( block_rows == (flip ? _cov.cols() : _cov.rows()) && block_cols == (flip ? _cov.rows() : _cov.cols()) && "Bad covariance size in WolfProblem::addCovarianceBlock");

    // STORE COVARIANCE
    for (unsigned int i = 0; i < block_rows; i++)
       for (unsigned int j = 0; j < block_cols; j++)
           covariance_.coeffRef(i+row,j+col) = (flip ? _cov(j,i) : _cov(i,j));
}

void RadarOdom+::getCovarianceBlock(StateBase* _state1, StateBase* _state2, Eigen::MatrixXs& _cov_block) const
{
    assert(_state1 != nullptr);
    assert(_state1->getPtr() != nullptr);
    assert(_state1->getPtr() < state_.data() + state_idx_last_);
    assert(_state1->getPtr() > state_.data());
    assert(_state2 != nullptr);
    assert(_state2->getPtr() != nullptr);
    assert(_state2->getPtr() < state_.data() + state_idx_last_);
    assert(_state2->getPtr() > state_.data());

    // Guarantee that we are getting the top triangular matrix (in cross covariance case)
    bool flip = _state1->getPtr() > _state2->getPtr();
    StateBase* stateA = (flip ? _state2 : _state1);
    StateBase* stateB = (flip ? _state1 : _state2);
    unsigned int row = (stateA->getPtr() - state_.data());
    unsigned int col = (stateB->getPtr() - state_.data());
    unsigned int block_rows = stateA->getStateSize();
    unsigned int block_cols = stateB->getStateSize();

    assert(_cov_block.rows() == (flip ? block_cols : block_rows) && _cov_block.cols() == (flip ? block_rows : block_cols) && "Bad _cov_block matrix sizes");

    _cov_block = (flip ? Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)) : Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)).transpose() );
}

void RadarOdom+::getCovarianceBlock(StateBase* _state1, StateBase* _state2, Eigen::Map<Eigen::MatrixXs>& _cov_block) const
{
    assert(_state1 != nullptr);
    assert(_state1->getPtr() != nullptr);
    assert(_state1->getPtr() < state_.data() + state_idx_last_);
    assert(_state1->getPtr() > state_.data());
    assert(_state2 != nullptr);
    assert(_state2->getPtr() != nullptr);
    assert(_state2->getPtr() < state_.data() + state_idx_last_);
    assert(_state2->getPtr() > state_.data());

    // Guarantee that we are getting the top triangular matrix (in cross covariance case)
    bool flip = _state1->getPtr() > _state2->getPtr();
    StateBase* stateA = (flip ? _state2 : _state1);
    StateBase* stateB = (flip ? _state1 : _state2);
    unsigned int row = (stateA->getPtr() - state_.data());
    unsigned int col = (stateB->getPtr() - state_.data());
    unsigned int block_rows = stateA->getStateSize();
    unsigned int block_cols = stateB->getStateSize();

    assert(_cov_block.rows() == (flip ? block_cols : block_rows) && _cov_block.cols() == (flip ? block_rows : block_cols) && "Bad _cov_block matrix sizes");

    _cov_block = (flip ? Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)).transpose() : Eigen::MatrixXs(covariance_.block(row, col, block_rows, block_cols)) );
}

void RadarOdom+::removeState(StateBase* _state_ptr)
{
	// TODO: Reordering? Mandatory for filtering...
	state_list_.remove(_state_ptr);
	removed_state_ptr_list_.push_back(_state_ptr->getPtr());
	delete _state_ptr;
}

WolfScalar* RadarOdom+::getStatePtr()
{
	return state_.data();
}

WolfScalar* RadarOdom+::getNewStatePtr()
{
	return state_.data()+state_idx_last_;
}

const unsigned int RadarOdom+::getStateSize() const
{
	return state_idx_last_;
}

void RadarOdom+::addMap(MapBase* _map_ptr)
{
	map_ptr_ = _map_ptr;
	map_ptr_->linkToUpperNode( this );
}

void RadarOdom+::addTrajectory(TrajectoryBase* _trajectory_ptr)
{
	trajectory_ptr_ = _trajectory_ptr;
	trajectory_ptr_->linkToUpperNode( this );
}

MapBase* RadarOdom+::getMapPtr()
{
	return map_ptr_;
}

TrajectoryBase* RadarOdom+::getTrajectoryPtr()
{
	return trajectory_ptr_;
}

FrameBase* RadarOdom+::getLastFramePtr()
{
    return trajectory_ptr_->getLastFramePtr();
}

StateBaseList* RadarOdom+::getStateListPtr()
{
	return &state_list_;
}

std::list<WolfScalar*>* RadarOdom+::getRemovedStateListPtr()
{
	return &removed_state_ptr_list_;
}

void RadarOdom+::print(unsigned int _ntabs, std::ostream& _ost) const
{
    printSelf(_ntabs, _ost); //one line
    printLower(_ntabs, _ost);
}

void RadarOdom+::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << nodeLabel() << " " << nodeId() << " : ";
    _ost << "TOP" << std::endl;
}

const Eigen::VectorXs RadarOdom+::getState() const
{
	return state_;
}

bool RadarOdom+::isReallocated() const
{
	return reallocated_;
}


void RadarOdom+::reallocationDone()
{
	reallocated_ = false;
}

RadarOdom+* RadarOdom+::getTop()
{
	return this;
}

void RadarOdom+::printLower(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << "\tLower Nodes  ==> [ ";
    _ost << map_ptr_->nodeId() << " ";
    _ost << trajectory_ptr_->nodeId() << " ]" << std::endl;
    _ntabs++;
	map_ptr_->print(_ntabs, _ost);
	trajectory_ptr_->print(_ntabs, _ost);
}
