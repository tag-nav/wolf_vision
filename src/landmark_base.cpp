
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "state_block.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
            NodeBase("LANDMARK", _type),
            map_ptr_(),
            state_block_vec_(4), // allow for 4 state blocks by default. Should be enough in all applications.
            landmark_id_(++landmark_id_count_),
            type_id_(_tp),
            status_(LANDMARK_CANDIDATE)//,
//			p_ptr_(_p_ptr),
//			o_ptr_(_o_ptr)
{
    //
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = nullptr;
    state_block_vec_[3] = nullptr;
    std::cout << "constructed  +L" << id() << std::endl;
}
                
LandmarkBase::~LandmarkBase()
{
    removeStateBlocks();
    std::cout << "destructed   -L" << id() << std::endl;
}

void LandmarkBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::cout << "Removing   L" << id() << std::endl;
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


void LandmarkBase::setStatus(LandmarkStatus _st)
{
    status_ = _st;

    // State Blocks
    if (status_ == LANDMARK_FIXED)
    {
        if (getPPtr()!=nullptr)
        {
            getPPtr()->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getPPtr());
        }
        if (getOPtr()!=nullptr)
        {
            getOPtr()->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getOPtr());
        }
    }
    else if(status_ == LANDMARK_ESTIMATED)
    {
        if (getPPtr()!=nullptr)
        {
            getPPtr()->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getPPtr());
        }
        if (getOPtr()!=nullptr)
        {
            getOPtr()->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getOPtr());
        }
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
        node["position"] = getPPtr()->getVector();
        node["position fixed"] = getPPtr()->isFixed();
    }
    if (getOPtr() != nullptr)
    {
        node["orientation"] = getOPtr()->getVector();
        node["orientation fixed"] = getOPtr()->isFixed();
    }
    return node;
}

} // namespace wolf
