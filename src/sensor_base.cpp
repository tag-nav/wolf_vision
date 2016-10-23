#include "sensor_base.h"
#include "state_block.h"


namespace wolf {

unsigned int SensorBase::sensor_id_count_ = 0;

SensorBase::SensorBase(const SensorType& _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr,
                       const unsigned int _noise_size, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(6), // allow for 6 state blocks by default. Should be enough in all applications.
        sensor_id_(++sensor_id_count_), // simple ID factory
        type_id_(_tp),
//        p_ptr_(_p_ptr),
//        o_ptr_(_o_ptr),
//        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_size),
        noise_cov_(_noise_size, _noise_size)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
    state_block_vec_[3] = nullptr;
    state_block_vec_[4] = nullptr;
    state_block_vec_[5] = nullptr;
    std::cout << "constructed  +S" << id() << std::endl;
    //
}

SensorBase::SensorBase(const SensorType & _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr,
                       const Eigen::VectorXs & _noise_std, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(6), // allow for 6 state blocks by default. Should be enough in all applications.
        sensor_id_(++sensor_id_count_), // simple ID factory
        type_id_(_tp),
//        p_ptr_(_p_ptr),
//        o_ptr_(_o_ptr),
//        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_std),
        noise_cov_(_noise_std.size(), _noise_std.size())
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
    state_block_vec_[3] = nullptr;
    state_block_vec_[4] = nullptr;
    state_block_vec_[5] = nullptr;
    noise_cov_.setZero();
    for (unsigned int i = 0; i < _noise_std.size(); i++)
        noise_cov_(i, i) = noise_std_(i) * noise_std_(i);

    std::cout << "constructed  +S" << id() << std::endl;
}

SensorBase::~SensorBase()
{
    // Remove State Blocks
    removeStateBlocks();
//    if (p_ptr_ != nullptr && !extrinsic_dynamic_)
//    {
//        if (getProblem() != nullptr)
//            getProblem()->removeStateBlockPtr(p_ptr_);
//        delete p_ptr_;
//        p_ptr_ = nullptr;
//    }
//
//    if (o_ptr_ != nullptr && !extrinsic_dynamic_)
//    {
//        if (getProblem() != nullptr)
//            getProblem()->removeStateBlockPtr(o_ptr_);
//        delete o_ptr_;
//        o_ptr_ = nullptr;
//    }
//
//    if (intrinsic_ptr_ != nullptr)
//    {
//        if (getProblem() != nullptr)
//            getProblem()->removeStateBlockPtr(intrinsic_ptr_);
//        delete intrinsic_ptr_;
//        intrinsic_ptr_ = nullptr;
//    }

    std::cout << "destructed   -S" << id() << std::endl;
}

inline void SensorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        SensorBasePtr this_S = shared_from_this(); // protect it while removing links

        // Remove State Blocks
        removeStateBlocks();
//        if (p_ptr_ != nullptr && !extrinsic_dynamic_)
//        {
//            if (getProblem() != nullptr)
//                getProblem()->removeStateBlockPtr(p_ptr_);
//            delete p_ptr_;
//            p_ptr_ = nullptr;
//        }
//        if (o_ptr_ != nullptr && !extrinsic_dynamic_)
//        {
//            if (getProblem() != nullptr)
//                getProblem()->removeStateBlockPtr(o_ptr_);
//            delete o_ptr_;
//            o_ptr_ = nullptr;
//        }
//        if (intrinsic_ptr_ != nullptr)
//        {
//            if (getProblem() != nullptr)
//                getProblem()->removeStateBlockPtr(intrinsic_ptr_);
//            delete intrinsic_ptr_;
//            intrinsic_ptr_ = nullptr;
//        }

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
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr && !extrinsic_dynamic_)
            {
                getProblem()->removeStateBlockPtr(sbp);
            }
            delete sbp;
            setStateBlockPtr(i, nullptr);
        }
    }
}



void SensorBase::fix()
{

    // fix only extrinsics
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

    // State Blocks
//    if (p_ptr_!=nullptr)
//    {
//        p_ptr_->fix();
//        if (getProblem() != nullptr)
//            getProblem()->updateStateBlockPtr(p_ptr_);
//    }
//    if (o_ptr_!=nullptr)
//    {
//        o_ptr_->fix();
//        if (getProblem() != nullptr)
//            getProblem()->updateStateBlockPtr(o_ptr_);
//    }
}

void SensorBase::unfix()
{
    // fix only extrinsics
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


    // State Blocks
//    if (p_ptr_!=nullptr)
//    {
//        p_ptr_->unfix();
//        if (getProblem() != nullptr)
//            getProblem()->updateStateBlockPtr(p_ptr_);
//    }
//    if (o_ptr_!=nullptr)
//    {
//        o_ptr_->unfix();
//        if (getProblem() != nullptr)
//            getProblem()->updateStateBlockPtr(o_ptr_);
//    }
}

void SensorBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }

//    if (getProblem() != nullptr)
//    {
//        if (p_ptr_ != nullptr)
//            getProblem()->addStateBlock(p_ptr_);
//
//        if (o_ptr_ != nullptr)
//            getProblem()->addStateBlock(o_ptr_);
//
//        if (intrinsic_ptr_ != nullptr)
//            getProblem()->addStateBlock(intrinsic_ptr_);
//    }
}

void SensorBase::setNoise(const Eigen::VectorXs& _noise_std) {
	noise_std_ = _noise_std;
	noise_cov_.setZero();
	for (unsigned int i=0; i<_noise_std.size(); i++)
		noise_cov_(i,i) = _noise_std(i) * _noise_std(i);
}

} // namespace wolf
