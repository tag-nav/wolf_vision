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

        static unsigned int landmark_id_count_;
        
    protected:
        unsigned int landmark_id_; ///< landmark unique id
        LandmarkType type_id_;     ///< type of landmark. (types defined at wolf.h)
        LandmarkStatus status_; ///< status of the landmark. (types defined at wolf.h)
        TimeStamp stamp_;       ///< stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
        StateBlock* p_ptr_;     ///< Position state block pointer
        StateBlock* o_ptr_;     ///< Orientation state block pointer
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
        LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);
        virtual ~LandmarkBase();
//        void destruct();
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

        /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        StateBlock* getPPtr() const;
        StateBlock* getOPtr() const;
        void setPPtr(StateBlock* _st_ptr);
        void setOPtr(StateBlock* _st_ptr);
        virtual std::vector<StateBlock*> getStateBlockVector() const;

        const Eigen::VectorXs& getDescriptor() const;        
        Scalar getDescriptor(unsigned int _ii) const;
        void setDescriptor(const Eigen::VectorXs& _descriptor);


        virtual YAML::Node saveToYaml() const;

        void addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList* getConstrainedByListPtr();
        /** \brief Remove the given constraint from the list.
         *  If list becomes empty, deletes this object by calling destruct()
         **/
        void removeConstrainedBy(ConstraintBasePtr _ctr_ptr);



        void setMapPtr(MapBasePtr _map_ptr){map_ptr_ = _map_ptr;}
        ProblemPtr getProblem();


};

}

#include "map_base.h"
#include "constraint_base.h"
#include "state_block.h"

namespace wolf{

inline void LandmarkBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::cout << "Removing   L" << id() << std::endl;
        LandmarkBasePtr this_L = shared_from_this();  // keep this alive while removing it

        // Remove State Blocks
        if (p_ptr_ != nullptr)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(p_ptr_);
            delete p_ptr_;
        }
        if (o_ptr_ != nullptr)
        {
            if (getProblem() != nullptr)
                getProblem()->removeStateBlockPtr(o_ptr_);
            delete o_ptr_;
        }

        // remove from upstream
        map_ptr_.lock()->getLandmarkListPtr()->remove(shared_from_this());

        // remove constrained by
        while (!constrained_by_list_.empty())
            constrained_by_list_.front()->remove();
    }
}

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

inline ConstraintBaseList* LandmarkBase::getConstrainedByListPtr()
{
    return &constrained_by_list_;
}

inline void LandmarkBase::removeConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);
    if (constrained_by_list_.empty())
        this->remove();
}

inline StateBlock* LandmarkBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* LandmarkBase::getOPtr() const
{
    return o_ptr_;
}

inline std::vector<StateBlock*> LandmarkBase::getStateBlockVector() const
{
    if (p_ptr_ == nullptr && o_ptr_ == nullptr)
        return std::vector<StateBlock*>(0);

    if (p_ptr_ == nullptr)
        return std::vector<StateBlock*>( {o_ptr_});

    if (o_ptr_ == nullptr)
        return std::vector<StateBlock*>( {p_ptr_});

    return std::vector<StateBlock*>( {p_ptr_, o_ptr_});
}

inline void LandmarkBase::setPPtr(StateBlock* _st_ptr)
{
    p_ptr_ = _st_ptr;
}

inline void LandmarkBase::setOPtr(StateBlock* _st_ptr)
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

//inline void LandmarkBase::destruct()
//{
//    if (!is_removing_)
//    {
//        if (map_ptr_ != nullptr) // && !up_node_ptr_->isTop())
//        {
//            //std::cout << "upper node is not WolfProblem " << std::endl;
//            map_ptr_->removeLandmark(shared_from_this());
//        }
//        else
//        {
//            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
//            //            delete this;//TODO remove line
//        }
//    }
//}

inline const LandmarkType LandmarkBase::getTypeId() const
{
    return type_id_;
}

} // namespace wolf
#endif
