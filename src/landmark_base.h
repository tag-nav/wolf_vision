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

        /** \brief Returns landmark_id_, the landmark unique id
         **/
        unsigned int id();
        void setId(unsigned int _id);
        const LandmarkType getTypeId() const;

        /** \brief Sets the Landmark status
         **/
        void setStatus(LandmarkStatus _st);

        /** \brief Sets the Landmark status to fixed
         **/
        void fix();

        /** \brief Sets the Landmark status to estimated
         **/
        void unfix();

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const
        {
            return state_block_vec_;
        }
        std::vector<StateBlockPtr>& getStateBlockVec()
        {
            return state_block_vec_;
        }
        std::vector<StateBlockPtr> getUsedStateBlockVec()
        {
            std::vector<StateBlockPtr> used_state_block_vec(0);
            for (auto sbp : state_block_vec_)
                if (sbp)
                    used_state_block_vec.push_back(sbp);
            return used_state_block_vec;
        }
        StateBlockPtr getStateBlockPtr(unsigned int _i) const
        {
            assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
            return state_block_vec_[_i];
        }
        void setStateBlockPtr(unsigned int _i, StateBlockPtr _sb_ptr)
        {
            state_block_vec_[_i] = _sb_ptr;
        }

        StateBlockPtr getPPtr() const;
        StateBlockPtr getOPtr() const;
        StateBlockPtr getVPtr() const;
        void setPPtr(StateBlockPtr _p_ptr);
        void setOPtr(StateBlockPtr _o_ptr);
        void setVPtr(StateBlockPtr _v_ptr);
        virtual void registerNewStateBlocks();
    protected:
        virtual void removeStateBlocks();

    public:
        const Eigen::VectorXs& getDescriptor() const;        
        Scalar getDescriptor(unsigned int _ii) const;
        void setDescriptor(const Eigen::VectorXs& _descriptor);


        virtual YAML::Node saveToYaml() const;

        void addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();

        void setMapPtr(MapBasePtr _map_ptr){map_ptr_ = _map_ptr;}
        MapBasePtr getMapPtr()
        {
            return map_ptr_.lock();
        }
        ProblemPtr getProblem();

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

inline StateBlockPtr LandmarkBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlockPtr LandmarkBase::getOPtr() const
{
    return o_ptr_;
}

inline void LandmarkBase::setPPtr(StateBlockPtr _st_ptr)
{
    p_ptr_ = _st_ptr;
}

inline void LandmarkBase::setOPtr(StateBlockPtr _st_ptr)
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

inline const LandmarkType LandmarkBase::getTypeId() const
{
    return type_id_;
}

} // namespace wolf
#endif
