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



// TODO: add descriptor as a StateBlock -> Could be estimated or not. Aperture could be one case of "descriptor"that can be estimated or not
// TODO: init and end Time stamps

//class LandmarkBase
class LandmarkBase : public NodeBase, public std::enable_shared_from_this<LandmarkBase>
{
    private:
        MapBaseWPtr map_ptr_;
        ConstraintBaseList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O.

        static unsigned int landmark_id_count_;
        
    protected:
        unsigned int landmark_id_; ///< landmark unique id
        LandmarkType type_id_;     ///< type of landmark. (types defined at wolf.h)
        LandmarkStatus status_; ///< status of the landmark. (types defined at wolf.h)
        TimeStamp stamp_;       ///< stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
        StateBlockPtr p_ptr_;     ///< Position state block pointer
        StateBlockPtr o_ptr_;     ///< Orientation state block pointer
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
        LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr);
        virtual ~LandmarkBase();
        void remove();
        virtual YAML::Node saveToYaml() const;

        // Properties
        unsigned int id();
        void setId(unsigned int _id);
        LandmarkType getTypeId() const;

        // Fix / unfix
        void setStatus(LandmarkStatus _st);
        void fix();
        void unfix();

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
        void addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();

        ProblemPtr getProblem();
        void setMapPtr(const MapBasePtr _map_ptr);
        MapBasePtr getMapPtr();

};

}

#include "map_base.h"
#include "constraint_base.h"
#include "state_block.h"

namespace wolf{

inline wolf::ProblemPtr LandmarkBase::getProblem()
{
    ProblemPtr prb = problem_ptr_.lock();
    if (!prb)
    {
        MapBasePtr map = map_ptr_.lock();
        if (map)
        {
            prb = map->getProblem();
            problem_ptr_ = prb;
        }
    }
    return prb;
}

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
    //std::cout << "Fixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_FIXED);
}

inline void LandmarkBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_ESTIMATED);
}

inline void LandmarkBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
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
    return p_ptr_;
}

inline StateBlockPtr LandmarkBase::getOPtr() const
{
    return o_ptr_;
}

inline void LandmarkBase::setPPtr(const StateBlockPtr _st_ptr)
{
    p_ptr_ = _st_ptr;
}

inline void LandmarkBase::setOPtr(const StateBlockPtr _st_ptr)
{
    o_ptr_ = _st_ptr;
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

inline LandmarkType LandmarkBase::getTypeId() const
{
    return type_id_;
}

} // namespace wolf
#endif
