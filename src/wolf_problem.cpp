#include "wolf_problem.h"

WolfProblem::WolfProblem() :
        NodeBase("WOLF_PROBLEM"), //
        location_(TOP)
{
}

WolfProblem::WolfProblem(WolfScalar* _state_ptr, const TrajectoryBaseShPtr& _trajectory_ptr, const MapBaseShPtr& _map_ptr) :
        NodeBase("WOLF_PROBLEM"), //
		state_ptr_(_state_ptr),
		state_idx_last_(0),
        location_(TOP),
		map_ptr_(_map_ptr),
		trajectory_ptr_(_trajectory_ptr)
{
}

WolfProblem::~WolfProblem()
{
}

WolfScalar* WolfProblem::getStatePtr()
{
	return state_ptr_;
}

const unsigned int WolfProblem::getStateIdx() const
{
	return state_idx_last_;
}

void WolfProblem::setStateIdx(unsigned int _idx)
{
	state_idx_last_ = _idx;
}

void WolfProblem::addMap(MapBaseShPtr& _map_ptr)
{
	map_ptr_ = _map_ptr;
}

void WolfProblem::addTrajectory(TrajectoryBaseShPtr& _trajectory_ptr)
{
	trajectory_ptr_ = _trajectory_ptr;
}

MapBasePtr WolfProblem::getMapPtr()
{
	return map_ptr_.get();
}

TrajectoryBasePtr WolfProblem::getTrajectoryPtr()
{
	return trajectory_ptr_.get();
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
