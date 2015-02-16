
#ifndef STATE_BASE_H_
#define STATE_BASE_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"

//class StateBase
class StateBase
{
    protected:
		WolfScalar* state_ptr_;
        
    public:
        /** \brief Constructor with whole state storage and index where starts the state unit
         * 
         * Constructor with whole state storage and index where starts the state unit
         * \param _st_remote is the whole state vector
         * \param _idx is the index of the first element of the state in the whole state vector
         * 
         **/
        StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx);

        /** \brief Constructor with scalar pointer
         * 
         * Constructor with scalar pointer
         * \param _st_ptr is the pointer of the first element of the state unit
         * 
         **/
        StateBase(WolfScalar* _st_ptr);
        
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
        
        /** \brief Returns the parametrization of the state unit
		 *
		 * Returns the parametrizationType (see wolf.h) of the state unit
		 *
		 **/
		virtual stateType getStateType() const = 0;

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
		virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const = 0;

		/** \brief prints a number of tabulations.
		 *
		 * Prints a number of tabulations, i.e., "\t". Inline function.
		 * \param _ntabs number of tabulations to print
		 * \param _ost output stream
		 *
		 */
		void printTabs(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
