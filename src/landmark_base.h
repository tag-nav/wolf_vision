#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

// Fwd references
class MapBase;
class NodeTerminus;

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_linked.h"
#include "state_block.h"

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

// TODO: add descriptor as a StateBase -> Could be estimated or not. Aperture could be one case of "descriptor"that can be estimated or not
// TODO: init and end Time stamps

//class LandmarkBase
class LandmarkBase : public NodeLinked<MapBase, NodeTerminus>
{
    protected:
        LandmarkType type_;     ///< type of landmark. (types defined at wolf.h)
        LandmarkStatus status_; ///< status of the landmark. (types defined at wolf.h)
        TimeStamp stamp_;       ///< stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
        StateBlock* p_ptr_;     ///< Position state unit pointer
        StateBlock* o_ptr_;     ///< Orientation state unit pointer
        Eigen::VectorXs descriptor_;    //TODO: agree?
        std::list<ConstraintBase*> constraint_to_list_; ///< List of constraints linked to this landmark

    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the position
         * \param _o_ptr StateBase pointer to the orientation (default: nullptr)
         *
         **/
        LandmarkBase(const LandmarkType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~LandmarkBase();

        /** \brief Link with a constraint
         *
         * Link with a constraint
         *
         **/
        void addConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Remove a constraint to this landmark
         *
         * Remove a constraint to this landmark
         *
         **/
        void removeConstraintTo(ConstraintBase* _ctr_ptr);

        /** \brief Gets the number of constraints linked with this landmark
         *
         * Gets the number of constraints linked with this landmark
         *
         **/
        unsigned int getHits() const;

        /** \brief Gets the list of constraints linked with this landmark
         *
         * Gets the list of constraints linked with this landmark
         *
         **/
        std::list<ConstraintBase*>* getConstraintToListPtr();

        /** \brief Sets the Landmark status
         *
         * Sets the Landmark status (see wolf.h)
         *
         **/
        void setStatus(LandmarkStatus _st);

        /** \brief Sets the Landmark status to fixed
         *
         * Sets the Landmark status to fixed
         *
         **/
        void fix();

        /** \brief Sets the Landmark status to estimated
         *
         * Sets the Landmark status to estimated
         *
         **/
        void unfix();

        /** \brief Gets the position state block pointer
         *
         * Gets the position state block pointer
         *
         **/
        StateBlock* getPPtr() const;

        /** \brief Gets the orientation state block pointer
         *
         * Gets the orientation state block pointer
         *
         **/
        StateBlock* getOPtr() const;

        /** \brief Sets the position state block pointer
         *
         * Sets the position state block pointer
         *
         **/
        void setPPtr(StateBlock* _st_ptr);

        /** \brief Sets the orientation state block pointer
         *
         * Sets the orientation state block pointer
         *
         **/
        void setOPtr(StateBlock* _st_ptr);

        /** \brief Sets the descriptor
         *
         * Sets the descriptor
         *
         **/
        void setDescriptor(const Eigen::VectorXs& _descriptor);

        /** \brief Gets the descriptor
         *
         * Gets the descriptor
         *
         **/
        const Eigen::VectorXs& getDescriptor() const;        
        
        /** \brief Returns _ii component of descriptor vector
         * 
         * Returns _ii component of descriptor_ vector
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         * 
         **/
        WolfScalar getDescriptor(unsigned int _ii) const;

        /** \brief Return the type of the landmark
         *
         * Return the type of the landmark (see wolf.h)
         *
         **/
        const LandmarkType getType() const;
};
#endif
