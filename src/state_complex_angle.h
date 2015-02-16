
#ifndef STATE_COMPLEX_ANGLE_H_
#define STATE_COMPLEX_ANGLE_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "state_base.h"

//class StateComplexAngle
class StateComplexAngle : public StateBase
{
    public:
		static const unsigned int BLOCK_SIZE = 2;

        /** \brief Constructor with whole state storage and index where starts the state unit
         * 
         * Constructor with whole state storage and index where starts the state unit
         * \param _st_remote is the whole state vector
         * \param _idx is the index of the first element of the state in the whole state vector
         * 
         **/
		StateComplexAngle(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /** \brief Constructor with scalar pointer
         * 
         * Constructor with scalar pointer
         * \param _st_ptr is the pointer of the first element of the state unit
         * 
         **/
		StateComplexAngle(WolfScalar* _st_ptr);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~StateComplexAngle();
        
        /** \brief Returns the parametrization of the state unit
		 *
		 * Returns the parametrizationType (see wolf.h) of the state unit
		 *
		 **/
		virtual stateType getStateType() const;

		/** \brief Returns the state unit size
		 *
		 * Returns the parametrizationType (see wolf.h) of the state unit
		 *
		 **/
		virtual unsigned int getStateSize() const;

        /** \brief Prints all the elements of the state unit
		 *
		 * Prints all the elements of the state unit
		 *
		 **/
		virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
