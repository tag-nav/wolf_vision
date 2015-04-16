
#ifndef STATE_BASE_H_
#define STATE_BASE_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//class StateBase
class StateBase : public NodeBase
{
    protected:
		WolfScalar* state_ptr_;
		StateStatus status_;
        
    public:
        /** \brief Constructor with scalar pointer
         * 
         * Constructor with scalar pointer
         * \param _st_ptr is the pointer of the first element of the state unit
         * 
         **/
        StateBase(WolfScalar* _st_ptr);

        /** \brief Constructor with whole state storage and index where starts the state unit
         * 
         * Constructor with whole state storage and index where starts the state unit
         * \param _st_remote is the whole state vector
         * \param _idx is the index of the first element of the state in the whole state vector
         * 
         **/
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~StateBase();
        
        /** \brief Returns the pointer to the first element of the state
		 *
		 * Returns the scalar pointer to the first element of the state
		 *
		 **/
        virtual WolfScalar* getPtr();
        
        /** \brief Returns a (mapped) vector of the state unit
         *
         * Returns a (mapped) vector of the state unit
         *
         **/
        virtual Eigen::Map<const Eigen::VectorXs> getVector() const = 0;

        /** \brief Set the pointer of the first element of the state
		 *
		 * Set the pointer of the first element of the state
		 *
		 **/
        virtual void setPtr(WolfScalar* _new_ptr);

        /** \brief Returns the parametrization of the state unit
		 *
		 * Returns the parametrizationType (see wolf.h) of the state unit
		 *
		 **/
		virtual StateType getStateType() const = 0;

        /** \brief Returns the status of the state unit
		 *
		 * Returns the stateStatus (see wolf.h) of the state unit
		 *
		 **/
		StateStatus getStateStatus() const;

		/** \brief Set the status of the state unit
		 *
		 * Set tthe stateStatus (see wolf.h) of the state unit
		 *
		 **/
		void setStateStatus(const StateStatus& _status);

		/** \brief Returns the state unit size
		 *
		 * Returns the parametrizationType (see wolf.h) of the state unit
		 *
		 **/
		virtual unsigned int getStateSize() const = 0;
        
        /** \brief Prints all the elements of the state unit
		 *
		 * Prints all the elements of the state unit
		 *
		 **/
		virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const = 0;
};
#endif
