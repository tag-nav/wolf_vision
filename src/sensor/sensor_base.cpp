#include "base/sensor/sensor_base.h"
#include "base/state_block.h"
#include "base/state_quaternion.h"
#include "base/factor/factor_block_absolute.h"
#include "base/factor/factor_quaternion_absolute.h"


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

void SensorBase::addPriorParameter(const unsigned int _i, const Eigen::VectorXs& _x, const Eigen::MatrixXs& _cov, unsigned int _start_idx, int _size)
{
    assert(!isStateBlockDynamic(_i) && "SensorBase::addPriorParameter only allowed for static parameters");
    assert(_i < state_block_vec_.size() && "State block not found");

    StateBlockPtr _sb = getStateBlockPtrStatic(_i);
    bool is_quaternion = (std::dynamic_pointer_cast<StateQuaternion>(_sb) != nullptr);

    assert(((!is_quaternion && _x.size() == _cov.rows() && _x.size() == _cov.cols()) ||
            (is_quaternion && _x.size() == 4 &&_cov.rows() == 3 && _cov.cols() == 3)) && "bad prior/covariance dimensions");
    assert((_size == -1 && _start_idx == 0) || (_size+_start_idx <= _sb->getSize()));
    assert(_size == -1 || _size == _x.size());
    assert(!(_size != -1 && is_quaternion) && "prior of a segment of state not available for quaternion");

    // set StateBlock state
    if (_size == -1)
        _sb->setState(_x);
    else
    {
        Eigen::VectorXs new_x = _sb->getState();
        new_x.segment(_start_idx,_size) = _x;
        _sb->setState(new_x);
    }

    // remove previous prior (if any)
    if (params_prior_map_.find(_i) != params_prior_map_.end())
        params_prior_map_[_i]->remove();

    // create feature
    FeatureBasePtr ftr_prior = std::make_shared<FeatureBase>("ABSOLUTE",_x,_cov);

    // set feature problem
    ftr_prior->setProblem(getProblem());

    // create & add factor absolute
    if (is_quaternion)
        ftr_prior->addFactor(std::make_shared<FactorQuaternionAbsolute>(_sb));
    else
        ftr_prior->addFactor(std::make_shared<FactorBlockAbsolute>(_sb, _start_idx, _size));

    // store feature in params_prior_map_
    params_prior_map_[_i] = ftr_prior;
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
    FrameBasePtrList frame_list = getProblem()->getTrajectory()->getFrameList();
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
    FrameBasePtrList frame_list = getProblem()->getTrajectory()->getFrameList();

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
    return getStateBlockPtr(0, _ts);
}

StateBlockPtr SensorBase::getOPtr(const TimeStamp _ts)
{
    return getStateBlockPtr(1, _ts);
}

StateBlockPtr SensorBase::getIntrinsicPtr(const TimeStamp _ts)
{
    return getStateBlockPtr(2, _ts);
}

StateBlockPtr SensorBase::getP()
{
    return getStateBlockPtr(0);
}

StateBlockPtr SensorBase::getO()
{
    return getStateBlockPtr(1);
}

StateBlockPtr SensorBase::getIntrinsic()
{
    return getStateBlockPtr(2);
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

StateBlockPtr SensorBase::getStateBlockPtr(unsigned int _i)
{
    CaptureBasePtr cap;

    if (isStateBlockDynamic(_i, cap))
        return cap->getStateBlockPtr(_i);

    return getStateBlockPtrStatic(_i);
}

StateBlockPtr SensorBase::getStateBlockPtr(unsigned int _i, const TimeStamp& _ts)
{
    CaptureBasePtr cap;

    if (isStateBlockDynamic(_i, _ts, cap))
        return cap->getStateBlockPtr(_i);

    return getStateBlockPtrStatic(_i);
}

bool SensorBase::isStateBlockDynamic(unsigned int _i, CaptureBasePtr& cap)
{
    if ((_i<2 && this->extrinsicsInCaptures()) || (_i>=2 && intrinsicsInCaptures()))
    {
        cap = lastKeyCapture();

        return cap != nullptr;
    }
    else
        return false;
}

bool SensorBase::isStateBlockDynamic(unsigned int _i, const TimeStamp& _ts, CaptureBasePtr& cap)
{
    if ((_i<2 && this->extrinsicsInCaptures()) || (_i>=2 && intrinsicsInCaptures()))
    {
        cap = lastCapture(_ts);

        return cap != nullptr;
    }
    else
        return false;
}

bool SensorBase::isStateBlockDynamic(unsigned int _i)
{
    CaptureBasePtr cap;

    return isStateBlockDynamic(_i,cap);
}

bool SensorBase::isStateBlockDynamic(unsigned int _i, const TimeStamp& _ts)
{
    CaptureBasePtr cap;

    return isStateBlockDynamic(_i,_ts,cap);
}

void SensorBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
    for (auto prc : processor_list_)
        prc->setProblem(_problem);
}

} // namespace wolf
