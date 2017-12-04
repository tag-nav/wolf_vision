
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "state_block.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

LandmarkBase::LandmarkBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
            NodeBase("LANDMARK", _type),
            map_ptr_(),
            state_block_vec_(2), // allow for 2 state blocks by default. Resize in derived constructors if needed.
            is_removing_(false),
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

void LandmarkBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
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
                getProblem()->removeStateBlockPtr(sbp);
            }
            setStateBlockPtr(i, nullptr);
        }
    }
}


YAML::Node LandmarkBase::saveToYaml() const
{
    YAML::Node node;
    node["id"] = landmark_id_;
    node["type"] = node_type_;
    if (getPPtr() != nullptr)
    {
        node["position"] = getPPtr()->getState();
        node["position fixed"] = getPPtr()->isFixed();
    }
    if (getOPtr() != nullptr)
    {
        node["orientation"] = getOPtr()->getState();
        node["orientation fixed"] = getOPtr()->isFixed();
    }
    return node;
}

} // namespace wolf
