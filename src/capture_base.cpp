#include "capture_base.h"
#include "sensor_base.h"

namespace wolf{

using namespace Eigen;

unsigned int CaptureBase::capture_id_count_ = 0;

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
    calib_size_(0),
    is_removing_(false),
    capture_id_(++capture_id_count_),
    time_stamp_(_ts)
{
    if (_sensor_ptr)
    {

        if (_sensor_ptr->isExtrinsicDynamic())
        {
            assert(_p_ptr && "Pointer to dynamic position params is null!");
            assert(_o_ptr && "Pointer to dynamic orientation params is null!");
            // assign to Capture's members
            state_block_vec_[0] = _p_ptr;
            state_block_vec_[1] = _o_ptr;
        }
        else if (_p_ptr || _o_ptr)
        {
            WOLF_ERROR("Provided dynamic sensor extrinsics but the sensor extrinsics are static");
        }

        if (_sensor_ptr->isIntrinsicDynamic())
        {
            assert(_intr_ptr && "Pointer to dynamic intrinsic params is null!");
            // assign to Capture's member
            state_block_vec_[2] = _intr_ptr;
        }
        else if (_intr_ptr)
        {
            WOLF_ERROR("Provided dynamic sensor intrinsics but the sensor intrinsics are static");
        }

        getSensorPtr()->setHasCapture();
        registerNewStateBlocks();
    }
    else if (_p_ptr || _o_ptr || _intr_ptr)
    {
        WOLF_ERROR("Provided sensor parameters but no sensor pointer");
    }
    updateCalibSize();
}



CaptureBase::~CaptureBase()
{
    removeStateBlocks();
}

void CaptureBase::remove()
{
    if (!is_removing_)
    {
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
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained by
        }
    }
}

FeatureBasePtr CaptureBase::addFeature(FeatureBasePtr _ft_ptr)
{
    //std::cout << "Adding feature" << std::endl;
    feature_list_.push_back(_ft_ptr);
    _ft_ptr->setCapturePtr(shared_from_this());
    _ft_ptr->setProblem(getProblem());
    return _ft_ptr;
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

StateBlockPtr CaptureBase::getStateBlockPtr(unsigned int _i) const
{
    if (getSensorPtr())
    {
        if (_i < 2) // _i == 0 is position, 1 is orientation, 2 and onwards are intrinsics
            if (getSensorPtr()->extrinsicsInCaptures())
            {
                assert(_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
                return state_block_vec_[_i];
            }
            else
                return getSensorPtr()->getStateBlockPtrStatic(_i);

        else // 2 and onwards are intrinsics
        if (getSensorPtr()->intrinsicsInCaptures())
        {
            assert(_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
            return state_block_vec_[_i];
        }
        else
            return getSensorPtr()->getStateBlockPtrStatic(_i);
    }
    else // No sensor associated: assume sensor params are here
    {
        assert(_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
        return state_block_vec_[_i];
    }
}

void CaptureBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sbp = state_block_vec_[i];
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
    for (Size i = 0; i<getStateBlockVec().size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
}

void CaptureBase::unfix()
{
    for (Size i = 0; i<getStateBlockVec().size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
}

void CaptureBase::fixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
}

void CaptureBase::unfixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
}

void CaptureBase::fixIntrinsics()
{
    for (unsigned int i = 2; i < getStateBlockVec().size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
}

void CaptureBase::unfixIntrinsics()
{
    for (unsigned int i = 2; i < getStateBlockVec().size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    }
    updateCalibSize();
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

wolf::Size CaptureBase::computeCalibSize() const
{
    Size sz = 0;
    for (Size i = 0; i < state_block_vec_.size(); i++)
    {
        auto sb = getStateBlockPtr(i);
        if (sb && !sb->isFixed())
            sz += sb->getSize();
    }
    return sz;
}

Eigen::VectorXs CaptureBase::getCalibration() const
{
    Eigen::VectorXs calib(calib_size_);
    Size index = 0;
    for (Size i = 0; i < getStateBlockVec().size(); i++)
    {
        auto sb = getStateBlockPtr(i);
        if (sb && !sb->isFixed())
        {
            calib.segment(index, sb->getSize()) = sb->getState();
            index += sb->getSize();
        }
    }
    return calib;
}

void CaptureBase::setCalibration(const VectorXs& _calib)
{
    updateCalibSize();
    assert(_calib.size() == calib_size_ && "Wrong size of calibration vector");
    Size index = 0;
    for (Size i = 0; i < getStateBlockVec().size(); i++)
    {
        auto sb = getStateBlockPtr(i);
        if (sb && !sb->isFixed())
        {
            sb->setState(_calib.segment(index, sb->getSize()));
            index += sb->getSize();
        }
    }
}




} // namespace wolf

