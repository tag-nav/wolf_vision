#include "capture_base.h"
#include "sensor_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr) :
        NodeBase("CAPTURE", _type),
        frame_ptr_(), // nullptr
        sensor_ptr_(_sensor_ptr),
        is_removing_(false),
        capture_id_(++capture_id_count_),
        time_stamp_(_ts)
{
    assert (_sensor_ptr->isExtrinsicDynamic() && "Sensor EX-trinsics dynamic. Please use constructor with extrinsics!");
    assert (_sensor_ptr->isIntrinsicDynamic() && "Sensor IN-trinsics dynamic. Please use constructor with intrinsics!");
//    std::cout << "constructed    +C" << id() << std::endl;
}

CaptureBase::CaptureBase(const std::string& _type,
                         const TimeStamp& _ts,
                         SensorBasePtr _sensor_ptr,
                         StateBlockPtr _p_ptr,
                         StateBlockPtr _o_ptr,
                         StateBlockPtr _intr_ptr) :
    NodeBase("CAPTURE", _type),
    frame_ptr_(), // nullptr
    sensor_ptr_(_sensor_ptr),
    state_block_vec_(3),
    is_removing_(false),
    capture_id_(++capture_id_count_),
    time_stamp_(_ts)
{
    if (_sensor_ptr->isExtrinsicDynamic())
    {
        assert(_p_ptr && _o_ptr && "At least one provided pointer to dynamic extrinsic params is null");
        assert((getSensorPtr()->getPPtr() == nullptr) && (getSensorPtr()->getOPtr() == nullptr) && "Sensor extrinsic pointers not null");
        state_block_vec_[0] = _p_ptr;
        state_block_vec_[1] = _o_ptr;
    }
    if (_sensor_ptr->isIntrinsicDynamic())
    {
        assert(_intr_ptr && "Provided pointer to dynamic intrinsic params is null");
        assert(getSensorIntrinsicPtr() == nullptr && "Sensor intrinsic pointer not null");
        state_block_vec_[2] = _intr_ptr;
    }
    registerNewStateBlocks();
    //    std::cout << "constructed    +C" << id() << std::endl;
}



CaptureBase::~CaptureBase()
{
    removeStateBlocks();
//    std::cout << "destructed     -C" << id() << std::endl;
}

void CaptureBase::remove()
{
//    std::cout << "Remove          C" << id() << std::endl;
    if (!is_removing_)
    {
//        std::cout << "Removing        C" << id() << std::endl;
        is_removing_ = true;
        CaptureBasePtr this_C = shared_from_this();  // keep this alive while removing it

        // Remove State Blocks
        removeStateBlocks();

        // remove from upstream
        FrameBasePtr F = frame_ptr_.lock();
        if (F)
        {
            F->getCaptureList().remove(this_C);
            if (F->getCaptureList().empty() && F->getConstrainedByList().empty())
                F->remove(); // remove upstream
        }

        // remove downstream
        while (!feature_list_.empty())
        {
            feature_list_.front()->remove(); // remove downstream
        }

//        std::cout << "Removed         C" << id() << std::endl;
    }
}

void CaptureBase::addFeatureList(FeatureBaseList& _new_ft_list)
{
    for (FeatureBasePtr feature_ptr : _new_ft_list)
    {
        feature_ptr->setCapturePtr(shared_from_this());
        if (getProblem() != nullptr)
            feature_ptr->setProblem(getProblem());
    }
    feature_list_.splice(feature_list_.end(), _new_ft_list);
}

void CaptureBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr)
            {
                getProblem()->removeStateBlockPtr(sbp);
            }
            setStateBlockPtr(i, nullptr);
        }
    }
}

void CaptureBase::fix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
}

void CaptureBase::unfix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
}

void CaptureBase::fixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
}

void CaptureBase::unfixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
}

void CaptureBase::fixIntrinsics()
{
    for (unsigned int i = 2; i < state_block_vec_.size(); i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
}

void CaptureBase::unfixIntrinsics()
{
    for (unsigned int i = 2; i < state_block_vec_.size(); i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
}



void CaptureBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
}




} // namespace wolf

