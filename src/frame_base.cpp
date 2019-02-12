
#include "base/frame_base.h"
#include "base/constraint/constraint_base.h"
#include "base/trajectory_base.h"
#include "base/capture/capture_base.h"
#include "base/state_block.h"
#include "base/state_angle.h"
#include "base/state_quaternion.h"

namespace wolf {

unsigned int FrameBase::frame_id_count_ = 0;

FrameBase::FrameBase(const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _v_ptr) :
            NodeBase("FRAME", "Base"),
            trajectory_ptr_(),
            state_block_vec_(3), // allow for 3 state blocks by default. Resize in derived constructors if needed.
            frame_id_(++frame_id_count_),
            type_(NON_KEY_FRAME),
            time_stamp_(_ts)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _v_ptr;
}

FrameBase::FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _v_ptr) :
            NodeBase("FRAME", "Base"),
            trajectory_ptr_(),
            state_block_vec_(3), // allow for 3 state blocks by default. Resize in derived constructors if needed.
            frame_id_(++frame_id_count_),
            type_(_tp),
            time_stamp_(_ts)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _v_ptr;
}
                
FrameBase::~FrameBase()
{
    if ( isKey() )
        removeStateBlocks();
}

void FrameBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        FrameBasePtr this_F = shared_from_this(); // keep this alive while removing it

        // remove from upstream
        TrajectoryBasePtr T = trajectory_ptr_.lock();
        if (T)
        {
            T->getFrameList().remove(this_F); // remove from upstream
        }

        // remove downstream
        while (!capture_list_.empty())
        {
            capture_list_.front()->remove(); // remove downstream
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
        }

        // Remove Frame State Blocks
        if ( isKey() )
            removeStateBlocks();

        if (getTrajectoryPtr()->getLastKeyFramePtr()->id() == this_F->id())
            getTrajectoryPtr()->setLastKeyFramePtr(getTrajectoryPtr()->findLastKeyFramePtr());

//        std::cout << "Removed       F" << id() << std::endl;
    }
}

void FrameBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
    if (isKey() && getTrajectoryPtr() != nullptr)
        getTrajectoryPtr()->sortFrame(shared_from_this());
}

void FrameBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (StateBlockPtr sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
}

void FrameBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        StateBlockPtr sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr)
            {
                getProblem()->removeStateBlock(sbp);
            }
            setStateBlockPtr(i, nullptr);
        }
    }
}

void FrameBase::setKey()
{
    if (type_ != KEY_FRAME)
    {
        type_ = KEY_FRAME;
        registerNewStateBlocks();

        if (getTrajectoryPtr()->getLastKeyFramePtr() == nullptr || getTrajectoryPtr()->getLastKeyFramePtr()->getTimeStamp() < time_stamp_)
            getTrajectoryPtr()->setLastKeyFramePtr(shared_from_this());

        getTrajectoryPtr()->sortFrame(shared_from_this());

//        WOLF_DEBUG("Set KF", this->id());
    }
}

void FrameBase::fix()
{
    for (auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->fix();
}

void FrameBase::unfix()
{
    for (auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->unfix();
}

bool FrameBase::isFixed() const
{
    bool fixed = true;
    for (auto sb : getStateBlockVec())
    {
        if (sb)
            fixed &= sb->isFixed();
    }
    return fixed;
}

void FrameBase::setState(const Eigen::VectorXs& _state)
{
    int size = 0;
    for(unsigned int i = 0; i<state_block_vec_.size(); i++){
        size += (state_block_vec_[i]==nullptr ? 0 : state_block_vec_[i]->getSize());
    }
    
    //State Vector size must be lower or equal to frame state size :
    // example : PQVBB -> if we initialize only position and orientation due to use of processorOdom3D
    assert(_state.size() <= size && "In FrameBase::setState wrong state size");

    unsigned int index = 0;
    const unsigned int _st_size = _state.size();

    //initialize the FrameBase StateBlocks while StateBlocks list's end not reached and input state_size can go further 
    for (StateBlockPtr sb : state_block_vec_)
        if (sb && (index < _st_size))
        {
            sb->setState(_state.segment(index, sb->getSize()), isKey());
            index += sb->getSize();
        }
}

Eigen::VectorXs FrameBase::getState() const
{
    SizeEigen size = 0;
    for (StateBlockPtr sb : state_block_vec_)
        if (sb)
            size += sb->getSize();
    Eigen::VectorXs state(size);

    getState(state);

    return state;
}

void FrameBase::getState(Eigen::VectorXs& _state) const
{
    SizeEigen size = 0;
    for (StateBlockPtr sb : state_block_vec_)
        if (sb)
            size += sb->getSize();

    assert(_state.size() == size && "Wrong state vector size");

    SizeEigen index = 0;

    for (StateBlockPtr sb : state_block_vec_)
        if (sb)
        {
            _state.segment(index,sb->getSize()) = sb->getState();
            index += sb->getSize();
        }
}

unsigned int FrameBase::getSize() const
{
    unsigned int size = 0;
    for (auto st : state_block_vec_)
        if (st)
            size += st->getSize();
    return size;
}

bool FrameBase::getCovariance(Eigen::MatrixXs& _cov) const
{
    return getProblem()->getFrameCovariance(shared_from_this(), _cov);
}

Eigen::MatrixXs FrameBase::getCovariance() const
{
    return getProblem()->getFrameCovariance(shared_from_this());
}

FrameBasePtr FrameBase::getPreviousFrame() const
{
    //std::cout << "finding previous frame of " << this->frame_id_ << std::endl;
    if (getTrajectoryPtr() == nullptr)
        //std::cout << "This Frame is not linked to any trajectory" << std::endl;

    assert(getTrajectoryPtr() != nullptr && "This Frame is not linked to any trajectory");

    //look for the position of this node in the upper list (frame list of trajectory)
    for (auto f_it = getTrajectoryPtr()->getFrameList().rbegin(); f_it != getTrajectoryPtr()->getFrameList().rend(); f_it++ )
    {
        if ( this->frame_id_ == (*f_it)->id() )
        {
        	f_it++;
        	if (f_it != getTrajectoryPtr()->getFrameList().rend())
            {
                //std::cout << "previous frame found!" << std::endl;
                return *f_it;
            }
        	else
        	{
        	    //std::cout << "previous frame not found!" << std::endl;
        	    return nullptr;
        	}
        }
    }
    //std::cout << "previous frame not found!" << std::endl;
    return nullptr;
}

FrameBasePtr FrameBase::getNextFrame() const
{
    //std::cout << "finding next frame of " << this->frame_id_ << std::endl;
	auto f_it = getTrajectoryPtr()->getFrameList().rbegin();
	f_it++; //starting from second last frame

    //look for the position of this node in the frame list of trajectory
    while (f_it != getTrajectoryPtr()->getFrameList().rend())
    {
        if ( this->frame_id_ == (*f_it)->id())
        {
        	f_it--;
			return *f_it;
        }
    	f_it++;
    }
    std::cout << "next frame not found!" << std::endl;
    return nullptr;
}

CaptureBasePtr FrameBase::addCapture(CaptureBasePtr _capt_ptr)
{
    capture_list_.push_back(_capt_ptr);
    _capt_ptr->setFramePtr(shared_from_this());
    _capt_ptr->setProblem(getProblem());
    _capt_ptr->registerNewStateBlocks();
    return _capt_ptr;
}

CaptureBasePtr FrameBase::getCaptureOf(const SensorBasePtr _sensor_ptr, const std::string& type)
{
    for (CaptureBasePtr capture_ptr : getCaptureList())
        if (capture_ptr->getSensorPtr() == _sensor_ptr && capture_ptr->getType() == type)
            return capture_ptr;
    return nullptr;
}

CaptureBasePtr FrameBase::getCaptureOf(const SensorBasePtr _sensor_ptr)
{
    for (CaptureBasePtr capture_ptr : getCaptureList())
        if (capture_ptr->getSensorPtr() == _sensor_ptr)
            return capture_ptr;
    return nullptr;
}

CaptureBaseList FrameBase::getCapturesOf(const SensorBasePtr _sensor_ptr)
{
    CaptureBaseList captures;
    for (CaptureBasePtr capture_ptr : getCaptureList())
        if (capture_ptr->getSensorPtr() == _sensor_ptr)
            captures.push_back(capture_ptr);
    return captures;
}

void FrameBase::unlinkCapture(CaptureBasePtr _cap_ptr)
{
    _cap_ptr->unlinkFromFrame();
    capture_list_.remove(_cap_ptr);
}

ConstraintBasePtr FrameBase::getConstraintOf(const ProcessorBasePtr _processor_ptr, const std::string& type)
{
    for (const ConstraintBasePtr& constaint_ptr : getConstrainedByList())
        if (constaint_ptr->getProcessor() == _processor_ptr && constaint_ptr->getType() == type)
            return constaint_ptr;
    return nullptr;
}

ConstraintBasePtr FrameBase::getConstraintOf(const ProcessorBasePtr _processor_ptr)
{
    for (const ConstraintBasePtr& constaint_ptr : getConstrainedByList())
        if (constaint_ptr->getProcessor() == _processor_ptr)
            return constaint_ptr;
    return nullptr;
}

void FrameBase::getConstraintList(ConstraintBaseList& _ctr_list)
{
    for (auto c_ptr : getCaptureList())
        c_ptr->getConstraintList(_ctr_list);
}

ConstraintBasePtr FrameBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setFrameOtherPtr(shared_from_this());
    return _ctr_ptr;
}

FrameBasePtr FrameBase::create_PO_2D(const FrameType & _tp,
                                     const TimeStamp& _ts,
                                     const Eigen::VectorXs& _x)
{
    assert(_x.size() == 3 && "Wrong state vector size. Should be 3 for 2D!");
    StateBlockPtr p_ptr ( std::make_shared<StateBlock>    (_x.head    <2> ( ) ) );
    StateBlockPtr o_ptr ( std::make_shared<StateAngle>    ((Scalar) _x(2) ) );
    StateBlockPtr v_ptr ( nullptr );
    FrameBasePtr f ( std::make_shared<FrameBase>(_tp, _ts, p_ptr, o_ptr, v_ptr) );
    f->setType("PO 2D");
    return f;
}
FrameBasePtr FrameBase::create_PO_3D(const FrameType & _tp,
                                     const TimeStamp& _ts,
                                     const Eigen::VectorXs& _x)
{
    assert(_x.size() == 7 && "Wrong state vector size. Should be 7 for 3D!");
    StateBlockPtr p_ptr ( std::make_shared<StateBlock>      (_x.head    <3> ( ) ) );
    StateBlockPtr o_ptr ( std::make_shared<StateQuaternion> (_x.tail    <4> ( ) ) );
    StateBlockPtr v_ptr ( nullptr );
    FrameBasePtr f ( std::make_shared<FrameBase>(_tp, _ts, p_ptr, o_ptr, v_ptr) );
    f->setType("PO 3D");
    return f;
}
FrameBasePtr FrameBase::create_POV_3D(const FrameType & _tp,
                                     const TimeStamp& _ts,
                                     const Eigen::VectorXs& _x)
{
    assert(_x.size() == 10 && "Wrong state vector size. Should be 10 for 3D!");
    StateBlockPtr p_ptr ( std::make_shared<StateBlock>      (_x.head    <3> ( ) ) );
    StateBlockPtr o_ptr ( std::make_shared<StateQuaternion> (_x.segment <4> (3) ) );
    StateBlockPtr v_ptr ( std::make_shared<StateBlock>      (_x.tail    <3> ( ) ) );
    FrameBasePtr f ( std::make_shared<FrameBase>(_tp, _ts, p_ptr, o_ptr, v_ptr) );
    f->setType("POV 3D");
    return f;
}

} // namespace wolf

#include "base/factory.h"
namespace wolf
{
namespace{ const bool WOLF_UNUSED Frame_PO_2D_Registered  = FrameFactory::get().registerCreator("PO 2D",  FrameBase::create_PO_2D ); }
namespace{ const bool WOLF_UNUSED Frame_PO_3D_Registered  = FrameFactory::get().registerCreator("PO 3D",  FrameBase::create_PO_3D ); }
namespace{ const bool WOLF_UNUSED Frame_POV_3D_Registered = FrameFactory::get().registerCreator("POV 3D", FrameBase::create_POV_3D); }
} // namespace wolf
