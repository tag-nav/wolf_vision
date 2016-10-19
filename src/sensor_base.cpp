#include "sensor_base.h"
#include "state_block.h"


namespace wolf {

unsigned int SensorBase::sensor_id_count_ = 0;

SensorBase::SensorBase(const SensorType& _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                       const unsigned int _noise_size, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        sensor_id_(++sensor_id_count_), // simple ID factory
        type_id_(_tp),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_size),
        noise_cov_(_noise_size, _noise_size)
{
    std::cout << "constructed  +S" << id() << std::endl;
    //
}

SensorBase::SensorBase(const SensorType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr,
                       const Eigen::VectorXs & _noise_std, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        sensor_id_(++sensor_id_count_), // simple ID factory
        type_id_(_tp),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        intrinsic_ptr_(_intr_ptr),
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_std),
        noise_cov_(_noise_std.size(), _noise_std.size())
{
    noise_cov_.setZero();
    for (unsigned int i = 0; i < _noise_std.size(); i++)
        noise_cov_(i, i) = noise_std_(i) * noise_std_(i);

    std::cout << "constructed  +S" << id() << std::endl;
}

SensorBase::~SensorBase()
{
    // Remove State Blocks
    if (p_ptr_ != nullptr && !extrinsic_dynamic_)
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(p_ptr_);
        delete p_ptr_;
        p_ptr_ = nullptr;
    }

    if (o_ptr_ != nullptr && !extrinsic_dynamic_)
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(o_ptr_);
        delete o_ptr_;
        o_ptr_ = nullptr;
    }

    if (intrinsic_ptr_ != nullptr)
    {
        if (getProblem() != nullptr)
            getProblem()->removeStateBlockPtr(intrinsic_ptr_);
        delete intrinsic_ptr_;
        intrinsic_ptr_ = nullptr;
    }

    std::cout << "destructed   -S" << id() << std::endl;
}

inline void SensorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        SensorBasePtr this_S = shared_from_this(); // protect it while removing links

        // Remove State Blocks
        if (p_ptr_ != nullptr && !extrinsic_dynamic_)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(p_ptr_);
            delete p_ptr_;
            p_ptr_ = nullptr;
        }
        if (o_ptr_ != nullptr && !extrinsic_dynamic_)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(o_ptr_);
            delete o_ptr_;
            o_ptr_ = nullptr;
        }
        if (intrinsic_ptr_ != nullptr)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(intrinsic_ptr_);
            delete intrinsic_ptr_;
            intrinsic_ptr_ = nullptr;
        }

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


void SensorBase::fix()
{
    // State Blocks
    if (p_ptr_!=nullptr)
    {
        p_ptr_->fix();
        if (getProblem() != nullptr)
            getProblem()->updateStateBlockPtr(p_ptr_);
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->fix();
        if (getProblem() != nullptr)
            getProblem()->updateStateBlockPtr(o_ptr_);
    }
}

void SensorBase::unfix()
{
    // State Blocks
    if (p_ptr_!=nullptr)
    {
        p_ptr_->unfix();
        if (getProblem() != nullptr)
            getProblem()->updateStateBlockPtr(p_ptr_);
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->unfix();
        if (getProblem() != nullptr)
            getProblem()->updateStateBlockPtr(o_ptr_);
    }
}

void SensorBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(p_ptr_);

        if (o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(o_ptr_);

        if (intrinsic_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(intrinsic_ptr_);
    }
}

void SensorBase::setNoise(const Eigen::VectorXs& _noise_std) {
	noise_std_ = _noise_std;
	noise_cov_.setZero();
	for (unsigned int i=0; i<_noise_std.size(); i++)
		noise_cov_(i,i) = _noise_std(i) * _noise_std(i);
}

} // namespace wolf
