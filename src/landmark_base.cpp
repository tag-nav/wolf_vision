
#include "landmark_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "state_block.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

unsigned int LandmarkBase::landmark_id_count_ = 0;

LandmarkBase::LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr) :
            NodeBase("LANDMARK", _type),
            map_ptr_(),
            landmark_id_(++landmark_id_count_),
            type_id_(_tp),
            status_(LANDMARK_CANDIDATE),
			p_ptr_(_p_ptr),
			o_ptr_(_o_ptr)
{
    //
    std::cout << "constructed  +L" << id() << std::endl;
}
                
LandmarkBase::~LandmarkBase()
{
//    remove();
    std::cout << "destructed   -L" << id() << std::endl;
}

void LandmarkBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::cout << "Removing   L" << id() << std::endl;
        LandmarkBasePtr this_L = shared_from_this(); // keep this alive while removing it
        // Remove State Blocks
        if (p_ptr_ != nullptr)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(p_ptr_);

            delete p_ptr_;
            p_ptr_ = nullptr;
        }
        if (o_ptr_ != nullptr)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(o_ptr_);

            delete o_ptr_;
            o_ptr_ = nullptr;
        }
        // remove from upstream
        map_ptr_.lock()->getLandmarkList().remove(shared_from_this());
        // remove constrained by
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove();
            constrained_by_list_.pop_front();
        }
    }
}

void LandmarkBase::setStatus(LandmarkStatus _st)
{
    status_ = _st;

    // State Blocks
    if (status_ == LANDMARK_FIXED)
    {
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
    else if(status_ == LANDMARK_ESTIMATED)
    {
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
}

void LandmarkBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(p_ptr_);
        if (o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(o_ptr_);
    }
}

YAML::Node LandmarkBase::saveToYaml() const
{
    YAML::Node node;
    node["id"] = landmark_id_;
    node["type"] = node_type_;
    if (p_ptr_ != nullptr)
    {
        node["position"] = p_ptr_->getVector();
        node["position fixed"] = p_ptr_->isFixed();
    }
    if (o_ptr_ != nullptr)
    {
        node["orientation"] = o_ptr_->getVector();
        node["orientation fixed"] = p_ptr_->isFixed();
    }
    return node;
}

} // namespace wolf
