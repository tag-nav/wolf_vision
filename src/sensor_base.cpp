#include "sensor_base.h"
#include "state_block.h"


namespace wolf {

unsigned int SensorBase::sensor_id_count_ = 0;

SensorBase::SensorBase(const std::string& _type,
                       StateBlockPtr _p_ptr,
                       StateBlockPtr _o_ptr,
                       StateBlockPtr _intr_ptr,
                       const unsigned int _noise_size,
                       const bool _extr_dyn,
                       const bool _intr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(3), // allow for 3 state blocks by default. Resize in derived constructors if needed.
        calib_size_(0),
        is_removing_(false),
        sensor_id_(++sensor_id_count_), // simple ID factory
        extrinsic_dynamic_(_extr_dyn),
        intrinsic_dynamic_(_intr_dyn),
        has_capture_(false),
        noise_std_(_noise_size),
        noise_cov_(_noise_size, _noise_size)
{
    noise_std_.setZero();
    noise_cov_.setZero();
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
    updateCalibSize();
}

SensorBase::SensorBase(const std::string& _type,
                       StateBlockPtr _p_ptr,
                       StateBlockPtr _o_ptr,
                       StateBlockPtr _intr_ptr,
                       const Eigen::VectorXs & _noise_std,
                       const bool _extr_dyn,
                       const bool _intr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(3), // allow for 3 state blocks by default. Resize in derived constructors if needed.
        calib_size_(0),
        is_removing_(false),
        sensor_id_(++sensor_id_count_), // simple ID factory
        extrinsic_dynamic_(_extr_dyn),
        intrinsic_dynamic_(_intr_dyn),
        has_capture_(false),
        noise_std_(_noise_std),
        noise_cov_(_noise_std.size(), _noise_std.size())
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
    setNoiseStd(_noise_std);
    updateCalibSize();
}

SensorBase::~SensorBase()
{
    // Remove State Blocks
    removeStateBlocks();
}

void SensorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        SensorBasePtr this_S = shared_from_this(); // protect it while removing links

        // Remove State Blocks
        removeStateBlocks();

        // remove from upstream
        auto H = hardware_ptr_.lock();
        if (H)
            H->getSensorList().remove(this_S);

        // remove downstream processors
        while (!processor_list_.empty())
        {
            processor_list_.front()->remove();
        }
    }

}

void SensorBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sbp = getStateBlockPtrStatic(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr && !extrinsic_dynamic_)
            {
                getProblem()->removeStateBlockPtr(sbp);
            }
            setStateBlockPtrStatic(i, nullptr);
        }
    }
}



void SensorBase::fix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    updateCalibSize();
}

void SensorBase::unfix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
    updateCalibSize();
}

void SensorBase::fixExtrinsics()
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
    updateCalibSize();
}

void SensorBase::unfixExtrinsics()
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
    updateCalibSize();
}

void SensorBase::fixIntrinsics()
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
    updateCalibSize();
}

void SensorBase::unfixIntrinsics()
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
    updateCalibSize();
}



void SensorBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
}

void SensorBase::setNoiseStd(const Eigen::VectorXs& _noise_std) {
    noise_std_ = _noise_std;
    noise_cov_.setZero();
    for (unsigned int i=0; i<_noise_std.size(); i++)
    {
        Scalar var_i = _noise_std(i) * _noise_std(i);
        noise_cov_(i,i) = var_i;
    }
}

void SensorBase::setNoiseCov(const Eigen::MatrixXs& _noise_cov) {
    noise_std_ = _noise_cov.diagonal().array().sqrt();
    noise_cov_ = _noise_cov;
}

CaptureBasePtr SensorBase::lastCapture(void)
{
    // we search for the most recent Capture of this sensor
    CaptureBasePtr capture = nullptr;
    FrameBaseList frame_list = getProblem()->getTrajectoryPtr()->getFrameList();
    FrameBaseRevIter frame_rev_it = frame_list.rbegin();
    while (frame_rev_it != frame_list.rend())
    {
        CaptureBasePtr capture = (*frame_rev_it)->getCaptureOf(shared_from_this());
        if (capture)
            // found the most recent Capture made by this sensor !
            break;
        frame_rev_it++;
    }
    return capture;
}

CaptureBasePtr SensorBase::lastCapture(const TimeStamp& _ts)
{
    // we search for the most recent Capture of this sensor before _ts
    CaptureBasePtr capture = nullptr;
    FrameBaseList frame_list = getProblem()->getTrajectoryPtr()->getFrameList();
    FrameBaseRevIter frame_rev_it = frame_list.rbegin();
    while (frame_rev_it != frame_list.rend())
    {
        if ((*frame_rev_it)->getTimeStamp() <= _ts)
        {
            CaptureBasePtr capture = (*frame_rev_it)->getCaptureOf(shared_from_this());
            if (capture)
                // found the most recent Capture made by this sensor !
                break;
        }
        frame_rev_it++;
    }
    return capture;
}

StateBlockPtr SensorBase::getPPtr(const TimeStamp _ts)
{
    return getStateBlockPtrDynamic(0, _ts);
}

StateBlockPtr SensorBase::getOPtr(const TimeStamp _ts)
{
    return getStateBlockPtrDynamic(1, _ts);
}

StateBlockPtr SensorBase::getIntrinsicPtr(const TimeStamp _ts)
{
    return getStateBlockPtrDynamic(2, _ts);
}

StateBlockPtr SensorBase::getPPtr()
{
    return getStateBlockPtrDynamic(0);
}

StateBlockPtr SensorBase::getOPtr()
{
    return getStateBlockPtrDynamic(1);
}

StateBlockPtr SensorBase::getIntrinsicPtr()
{
    return getStateBlockPtrDynamic(2);
}

wolf::Size SensorBase::computeCalibSize() const
{
    Size sz = 0;
    for (Size i = 0; i < state_block_vec_.size(); i++)
    {
        auto sb = state_block_vec_[i];
        if (sb && !sb->isFixed())
            sz += sb->getSize();
    }
    return sz;
}


Eigen::VectorXs SensorBase::getCalibration() const
{
    Size index = 0;
    Size sz = getCalibSize();
    Eigen::VectorXs calib(sz);
    for (Size i = 0; i < state_block_vec_.size(); i++)
    {
        auto sb = getStateBlockPtrStatic(i);
        if (sb && !sb->isFixed())
        {
            calib.segment(index, sb->getSize()) = sb->getState();
            index += sb->getSize();
        }
    }
    return calib;
}


bool SensorBase::process(const CaptureBasePtr capture_ptr)
{
    capture_ptr->setSensorPtr(shared_from_this());

    for (const auto processor : processor_list_)
    {
        processor->process(capture_ptr);
    }

    return true;
}

ProcessorBasePtr SensorBase::addProcessor(ProcessorBasePtr _proc_ptr)
{
    processor_list_.push_back(_proc_ptr);
    _proc_ptr->setSensorPtr(shared_from_this());
    _proc_ptr->setProblem(getProblem());
    return _proc_ptr;
}

StateBlockPtr SensorBase::getStateBlockPtrDynamic(unsigned int _i)
{
    if ((_i<2 && this->extrinsicsInCaptures()) || (_i>=2 && intrinsicsInCaptures()))
    {
        CaptureBasePtr cap = lastCapture();
        if (cap)
            return cap->getStateBlockPtr(_i);
        else
            return getStateBlockPtrStatic(_i);
    }
    else
        return getStateBlockPtrStatic(_i);
}

StateBlockPtr SensorBase::getStateBlockPtrDynamic(unsigned int _i, const TimeStamp& _ts)
{
    if ((_i<2 && this->extrinsicsInCaptures()) || (_i>=2 && intrinsicsInCaptures()))
    {
        CaptureBasePtr cap = lastCapture(_ts);
        if (cap)
            return cap->getStateBlockPtr(_i);
        else
            return getStateBlockPtrStatic(_i);
    }
    else
        return getStateBlockPtrStatic(_i);
}

} // namespace wolf
