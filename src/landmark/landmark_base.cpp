
#include "base/landmark/landmark_base.h"
#include "base/factor/factor_base.h"
#include "base/map_base.h"
#include "base/state_block.h"
#include "base/yaml/yaml_conversion.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

    LandmarkBase::LandmarkBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
            NodeBase("LANDMARK", _type),
            map_ptr_(),
            state_block_vec_(2), // allow for 2 state blocks by default. Resize in derived constructors if needed.
            landmark_id_(++landmark_id_count_)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;

//    std::cout << "constructed  +L" << id() << std::endl;
}

    LandmarkBase::LandmarkBase(MapBaseWPtr _ptr, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
        NodeBase("LANDMARK", _type),
        map_ptr_(_ptr),
        state_block_vec_(2), // allow for 2 state blocks by default. Resize in derived constructors if needed.
        landmark_id_(++landmark_id_count_)
    {
        state_block_vec_[0] = _p_ptr;
        state_block_vec_[1] = _o_ptr;

        //    std::cout << "constructed  +L" << id() << std::endl;
    }
LandmarkBase::~LandmarkBase()
{
    removeStateBlocks();
//    std::cout << "destructed   -L" << id() << std::endl;
}

void LandmarkBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        LandmarkBasePtr this_L = shared_from_this(); // keep this alive while removing it

        // remove from upstream
        auto M = map_ptr_.lock();
        if (M)
            M->getLandmarkList().remove(shared_from_this());

        // remove constrained by
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove();
        }

        // Remove State Blocks
        removeStateBlocks();
    }
}

void LandmarkBase::fix()
{
    for (auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->fix();
}

void LandmarkBase::unfix()
{
    for (auto sbp : state_block_vec_)
        if (sbp != nullptr)
            sbp->unfix();
}

bool LandmarkBase::isFixed() const
{
    bool fixed = true;
    for (auto sb : getStateBlockVec())
    {
        if (sb)
            fixed &= sb->isFixed();
    }
    return fixed;
}

std::vector<StateBlockPtr> LandmarkBase::getUsedStateBlockVec() const
{
    std::vector<StateBlockPtr> used_state_block_vec(0);
    for (auto sbp : state_block_vec_)
        if (sbp)
            used_state_block_vec.push_back(sbp);
    return used_state_block_vec;
}

void LandmarkBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
}

Eigen::MatrixXs LandmarkBase::getCovariance() const
{
    return getProblem()->getLandmarkCovariance(shared_from_this());
}

bool LandmarkBase::getCovariance(Eigen::MatrixXs& _cov) const
{
    return getProblem()->getLandmarkCovariance(shared_from_this(), _cov);
}

void LandmarkBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
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

Eigen::VectorXs LandmarkBase::getState() const
{
    SizeEigen size = 0;
    for (StateBlockPtr sb : state_block_vec_)
        if (sb)
            size += sb->getSize();
    Eigen::VectorXs state(size);

    getState(state);

    return state;
}

void LandmarkBase::getState(Eigen::VectorXs& _state) const
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

YAML::Node LandmarkBase::saveToYaml() const
{
    YAML::Node node;
    node["id"] = landmark_id_;
    node["type"] = node_type_;
    if (getP() != nullptr)
    {
        node["position"] = getP()->getState();
        node["position fixed"] = getP()->isFixed();
    }
    if (getO() != nullptr)
    {
        node["orientation"] = getO()->getState();
        node["orientation fixed"] = getO()->isFixed();
    }
    return node;
}
    void LandmarkBase::link(MapBasePtr _map_ptr)
    {
        std::cout << "Linking LandmarkBase" << std::endl;
        // this->map_ptr_.lock()->addLandmark(shared_from_this());
        this->setMapPtr(_map_ptr);
        this->map_ptr_.lock()->addLandmark(shared_from_this());
    }
FactorBasePtr LandmarkBase::addConstrainedBy(FactorBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setLandmarkOtherPtr(shared_from_this());
    return _ctr_ptr;
}

} // namespace wolf
