#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

// Fwd references
namespace wolf{
class MapBase;
class NodeTerminus;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "node_constrained.h"

//std includes

namespace wolf {

// TODO: add descriptor as a StateBlock -> Could be estimated or not. Aperture could be one case of "descriptor"that can be estimated or not
// TODO: init and end Time stamps

//class LandmarkBase
class LandmarkBase : public NodeConstrained<MapBase, NodeTerminus>
{
    private:
        static unsigned int landmark_id_count_;
        
    protected:
        unsigned int landmark_id_; ///< landmark unique id
        LandmarkType type_id_;     ///< type of landmark. (types defined at wolf.h)
        LandmarkStatus status_; ///< status of the landmark. (types defined at wolf.h)
        TimeStamp stamp_;       ///< stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
        StateBlock* p_ptr_;     ///< Position state unit pointer
        StateBlock* o_ptr_;     ///< Orientation state unit pointer
        Eigen::VectorXs descriptor_;    //TODO: agree? JS: No: It is not general enough as descriptor to be in LmkBase.
        ConstraintBaseList constrained_by_list_; ///< List of constraints linked to this landmark


    public:

        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the position
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
        LandmarkBase(const LandmarkType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~LandmarkBase();

        /** \brief Returns landmark_id_, the landmark unique id
         **/
        unsigned int id();

        /** \brief Sets the Landmark status
         **/
        void setStatus(LandmarkStatus _st);

        /** \brief Sets the Landmark status to fixed
         **/
        void fix();

        /** \brief Sets the Landmark status to estimated
         **/
        void unfix();

        /** \brief Remove the given constraint from the list. 
         *  If list becomes empty, deletes this object by calling destruct()
         **/
        void removeConstrainedBy(ConstraintBase* _ctr_ptr);

        /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        /** \brief Gets the position state block pointer
         **/
        StateBlock* getPPtr() const;

        /** \brief Gets the orientation state block pointer
         **/
        StateBlock* getOPtr() const;

        /** \brief Sets the position state block pointer
         **/
        void setPPtr(StateBlock* _st_ptr);

        /** \brief Sets the orientation state block pointer
         **/
        void setOPtr(StateBlock* _st_ptr);

        /** \brief Sets the descriptor
         **/
        void setDescriptor(const Eigen::VectorXs& _descriptor);

        /** \brief Gets the descriptor
         **/
        const Eigen::VectorXs& getDescriptor() const;        
        
        /** \brief Returns _ii component of descriptor vector
         * 
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         **/
        Scalar getDescriptor(unsigned int _ii) const;

        /** \brief Return the type of the landmark
         **/
        const LandmarkType getType() const;
};

inline unsigned int LandmarkBase::id()
{
    return landmark_id_;
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

inline void LandmarkBase::removeConstrainedBy(ConstraintBase* _ctr_ptr)
{
    NodeConstrained::removeConstrainedBy(_ctr_ptr);
    if (constrained_by_list_.empty())
        this->destruct();
}

inline StateBlock* LandmarkBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* LandmarkBase::getOPtr() const
{
    return o_ptr_;
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
    return descriptor_(_ii);
}

inline const Eigen::VectorXs& LandmarkBase::getDescriptor() const
{
    return descriptor_;
}

inline const LandmarkType LandmarkBase::getType() const
{
    return type_id_;
}

} // namespace wolf
#endif
