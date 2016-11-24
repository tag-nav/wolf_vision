#include "sensor_base.h"
#include "state_block.h"


namespace wolf {

unsigned int SensorBase::sensor_id_count_ = 0;

SensorBase::SensorBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr,
                       const unsigned int _noise_size, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(3), // allow for 3 state blocks by default. Resize in derived constructors if needed.
        sensor_id_(++sensor_id_count_), // simple ID factory
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_size),
        noise_cov_(_noise_size, _noise_size)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
//    std::cout << "constructed  +S" << id() << std::endl;
    //
}

SensorBase::SensorBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _intr_ptr,
                       const Eigen::VectorXs & _noise_std, const bool _extr_dyn) :
        NodeBase("SENSOR", _type),
        hardware_ptr_(),
        state_block_vec_(6), // allow for 3 state blocks by default. Resize in derived constructors if needed.
        sensor_id_(++sensor_id_count_), // simple ID factory
        extrinsic_dynamic_(_extr_dyn),
        noise_std_(_noise_std),
        noise_cov_(_noise_std.size(), _noise_std.size())
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _intr_ptr;
    noise_cov_.setZero();
    for (unsigned int i = 0; i < _noise_std.size(); i++)
        noise_cov_(i, i) = noise_std_(i) * noise_std_(i);

//    std::cout << "constructed  +S" << id() << std::endl;
}

SensorBase::~SensorBase()
{
    // Remove State Blocks
    removeStateBlocks();

//    std::cout << "destructed   -S" << id() << std::endl;
}

inline void SensorBase::remove()
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
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr && !extrinsic_dynamic_)
            {
                getProblem()->removeStateBlockPtr(sbp);
            }
            setStateBlockPtr(i, nullptr);
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

void SensorBase::setNoise(const Eigen::VectorXs& _noise_std) {
	noise_std_ = _noise_std;
	noise_cov_.setZero();
	for (unsigned int i=0; i<_noise_std.size(); i++)
		noise_cov_(i,i) = _noise_std(i) * _noise_std(i);
}

} // namespace wolf
