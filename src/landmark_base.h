#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

// Fwd references
namespace wolf{
class MapBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "time_stamp.h"

//std includes

// yaml
#include "yaml-cpp/yaml.h"

namespace wolf {


//class LandmarkBase
class LandmarkBase : public NodeBase, public std::enable_shared_from_this<LandmarkBase>
{
    private:
        MapBaseWPtr map_ptr_;
        ConstraintBaseList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O.

        static unsigned int landmark_id_count_;
        bool is_removing_; ///< A flag for safely removing nodes from the Wolf tree. See remove().

    protected:
        unsigned int landmark_id_; ///< landmark unique id
        TimeStamp stamp_;       ///< stamp of the creation of the landmark
        Eigen::VectorXs descriptor_;    //TODO: agree? JS: No: It is not general enough as descriptor to be in LmkBase.

    public:

        /** \brief Constructor with type, time stamp and the position state pointer (optional orientation state pointer)
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the position
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
        LandmarkBase(const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr);
        virtual ~LandmarkBase();
        void remove();
        virtual YAML::Node saveToYaml() const;

        // Properties
        unsigned int id();
        void setId(unsigned int _id);

        // Fix / unfix
        void fix();
        void unfix();
        bool isFixed() const;

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        std::vector<StateBlockPtr> getUsedStateBlockVec() const;
        StateBlockPtr getStateBlockPtr(unsigned int _i) const;
        void setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr);
        StateBlockPtr getPPtr() const;
        StateBlockPtr getOPtr() const;
        StateBlockPtr getVPtr() const;
        void setPPtr(const StateBlockPtr _p_ptr);
        void setOPtr(const StateBlockPtr _o_ptr);
        void setVPtr(const StateBlockPtr _v_ptr);
        virtual void registerNewStateBlocks();
    protected:
        virtual void removeStateBlocks();

        // Descriptor
    public:
        const Eigen::VectorXs& getDescriptor() const;        
        Scalar getDescriptor(unsigned int _ii) const;
        void setDescriptor(const Eigen::VectorXs& _descriptor);


        // Navigate wolf tree
        ConstraintBasePtr addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();

        void setMapPtr(const MapBasePtr _map_ptr);
        MapBasePtr getMapPtr();

};

}

#include "map_base.h"
#include "constraint_base.h"
#include "state_block.h"

namespace wolf{

inline MapBasePtr LandmarkBase::getMapPtr()
{
    return map_ptr_.lock();
}

inline void LandmarkBase::setMapPtr(const MapBasePtr _map_ptr)
{
    map_ptr_ = _map_ptr;
}

inline unsigned int LandmarkBase::id()
{
    return landmark_id_;
}

inline void LandmarkBase::setId(unsigned int _id)
{
    landmark_id_ = _id;
    if (_id > landmark_id_count_)
        landmark_id_count_ = _id;
}

inline void LandmarkBase::fix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
}

inline void LandmarkBase::unfix()
{
    for( auto sbp : state_block_vec_)
        if (sbp != nullptr)
        {
            sbp->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(sbp);
        }
}

inline bool LandmarkBase::isFixed() const
{
    bool fixed = true;
    for (auto sb : getStateBlockVec())
    {
        if (sb)
            fixed &= sb->isFixed();
    }
    return fixed;
}

inline ConstraintBasePtr LandmarkBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
    _ctr_ptr->setLandmarkOtherPtr( shared_from_this() );
    return _ctr_ptr;
}

inline unsigned int LandmarkBase::getHits() const
{
    return constrained_by_list_.size();
}

inline ConstraintBaseList& LandmarkBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline const std::vector<StateBlockPtr>& LandmarkBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& LandmarkBase::getStateBlockVec()
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr> LandmarkBase::getUsedStateBlockVec() const
{
    std::vector<StateBlockPtr> used_state_block_vec(0);
    for (auto sbp : state_block_vec_)
        if (sbp)
            used_state_block_vec.push_back(sbp);
    return used_state_block_vec;
}

inline StateBlockPtr LandmarkBase::getStateBlockPtr(unsigned int _i) const
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    return state_block_vec_[_i];
}

inline void LandmarkBase::setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr)
{
    state_block_vec_[_i] = _sb_ptr;
}

inline StateBlockPtr LandmarkBase::getPPtr() const
{
    return getStateBlockPtr(0);
}

inline StateBlockPtr LandmarkBase::getOPtr() const
{
    return getStateBlockPtr(1);
}

inline void LandmarkBase::setPPtr(const StateBlockPtr _st_ptr)
{
    setStateBlockPtr(0, _st_ptr);
}

inline void LandmarkBase::setOPtr(const StateBlockPtr _st_ptr)
{
    setStateBlockPtr(1, _st_ptr);
}

inline void LandmarkBase::setDescriptor(const Eigen::VectorXs& _descriptor)
{
    descriptor_ = _descriptor;
}

inline Scalar LandmarkBase::getDescriptor(unsigned int _ii) const
{
    assert(_ii < descriptor_.size() && "LandmarkBase::getDescriptor: bad index");
    return descriptor_(_ii);
}

inline const Eigen::VectorXs& LandmarkBase::getDescriptor() const
{
    return descriptor_;
}

} // namespace wolf
#endif
