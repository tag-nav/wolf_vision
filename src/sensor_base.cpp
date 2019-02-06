#include "sensor_base.h"
#include "state_block.h"
#include "constraint_block_absolute.h"
#include "constraint_quaternion_absolute.h"


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
                getProblem()->removeStateBlock(sbp);
            }
            setStateBlockPtrStatic(i, nullptr);
        }
    }
}



void SensorBase::fix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->fix();
    updateCalibSize();
}

void SensorBase::unfix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->unfix();
    updateCalibSize();
}

void SensorBase::fixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
            sbp->fix();
    }
    updateCalibSize();
}

void SensorBase::unfixExtrinsics()
{
    for (unsigned int i = 0; i < 2; i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
            sbp->unfix();
    }
    updateCalibSize();
}

void SensorBase::fixIntrinsics()
{
    for (unsigned int i = 2; i < state_block_vec_.size(); i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
            sbp->fix();
    }
    updateCalibSize();
}

void SensorBase::unfixIntrinsics()
{
    for (unsigned int i = 2; i < state_block_vec_.size(); i++)
    {
        auto sbp = state_block_vec_[i];
        if (sbp != nullptr)
            sbp->unfix();
    }
    updateCalibSize();
}

void SensorBase::addParameterPrior(const StateBlockPtr& _sb, const Eigen::VectorXs& _x, const Eigen::MatrixXs& _cov, unsigned int _start_idx = 0, int _size = -1)
{
    assert(std::find(state_block_vec_.begin(),state_block_vec_.end(),_sb) != state_block_vec_.end() && "adding prior to unknown state block");
    assert(_x.size() == _cov.rows() && _x.size() == _cov.cols() && "covariance and prior dimension should be the same");
    assert((_size == -1 && _start_idx == 0) || (_size+_start_idx <= _sb->getSize()));
    assert(_size == -1 || _size == _x.size());

    // create feature
    auto ftr_prior = std::make_shared<FeatureBase>(_x,_cov);

    // set feature problem
    ftr_prior->setProblem(getProblem());

    // create & add constraint absolute
    if (std::dynamic_pointer_cast<StateQuaternion>(_sb))
        ftr_prior->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(_sb));
    else
        ftr_prior->addConstraint(std::make_shared<ConstraintBlockAbsolute>(_sb, _start_idx, _size));
}

void SensorBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (unsigned int i = 0; i < getStateBlockVec().size(); i++)
        {
            if (i < 2 && !isExtrinsicDynamic())
            {
                auto sbp = getStateBlockPtrStatic(i);
                if (sbp != nullptr)
                    getProblem()->addStateBlock(sbp);
            }
            if (i >= 2 && !isIntrinsicDynamic())
            {
                auto sbp = getStateBlockPtrStatic(i);
                if (sbp != nullptr)
                    getProblem()->addStateBlock(sbp);
            }
        }
    }
}

void SensorBase::setNoiseStd(const Eigen::VectorXs& _noise_std) {
    noise_std_ = _noise_std;
    noise_cov_ = _noise_std.array().square().matrix().asDiagonal();
}

void SensorBase::setNoiseCov(const Eigen::MatrixXs& _noise_cov) {
    noise_std_ = _noise_cov.diagonal().array().sqrt();
    noise_cov_ = _noise_cov;
}

CaptureBasePtr SensorBase::lastKeyCapture(void)
{
    // we search for the most recent Capture of this sensor which belongs to a KeyFrame
    CaptureBasePtr capture = nullptr;
    FrameBaseList frame_list = getProblem()->getTrajectoryPtr()->getFrameList();
    FrameBaseRevIter frame_rev_it = frame_list.rbegin();
    while (frame_rev_it != frame_list.rend())
    {
        if ((*frame_rev_it)->isKey())
        {
            capture = (*frame_rev_it)->getCaptureOf(shared_from_this());
            if (capture)
                // found the most recent Capture made by this sensor !
                break;
        }
        frame_rev_it++;
    }
    return capture;
}

CaptureBasePtr SensorBase::lastCapture(const TimeStamp& _ts)
{
    // we search for the most recent Capture of this sensor before _ts
    CaptureBasePtr capture = nullptr;
    FrameBaseList frame_list = getProblem()->getTrajectoryPtr()->getFrameList();

    // We iterate in reverse since we're likely to find it close to the rbegin() place.
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

SizeEigen SensorBase::computeCalibSize() const
{
    SizeEigen sz = 0;
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sb = state_block_vec_[i];
        if (sb && !sb->isFixed())
            sz += sb->getSize();
    }
    return sz;
}


Eigen::VectorXs SensorBase::getCalibration() const
{
    SizeEigen index = 0;
    SizeEigen sz = getCalibSize();
    Eigen::VectorXs calib(sz);
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
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
        CaptureBasePtr cap = lastKeyCapture();
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

void SensorBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
    for (auto prc : processor_list_)
        prc->setProblem(_problem);
}

} // namespace wolf
