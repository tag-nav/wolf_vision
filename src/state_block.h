
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//class StateBlock
class StateBlock
{
    protected:
		StateType type_;
		Eigen::VectorXs state_;
		bool fixed_;
        
    public:
        /** \brief Constructor with scalar pointer
         * 
         * Constructor with scalar pointer
         * \param _state is state vector
         * \param _type parametrization of the state
         * 
         **/
        StateBlock(const Eigen::VectorXs _state, StateType _type = ST_VECTOR, bool _fixed = false);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~StateBlock();
        
        /** \brief Returns the pointer to the first element of the state
		 *
		 * Returns the scalar pointer to the first element of the state
		 *
		 **/
        WolfScalar* getPtr();
        
        /** \brief Returns the state vector
         *
         * Returns the state vector
         *
         **/
        Eigen::VectorXs getVector();

        /** \brief Sets the state vector
         *
         * Sets the state vector
         *
         **/
        void setVector(const Eigen::VectorXs& _state);

        /** \brief Returns the parametrization of the state
		 *
		 * Returns the parametrizationType (see wolf.h) of the state
		 *
		 **/
		virtual StateType getType() const;

        /** \brief Returns the state size
		 *
		 * Returns the state size
		 *
		 **/
		unsigned int getSize() const;

        /** \brief Returns if the state is fixed (not estimated)
         *
         * Returns if the state is fixed (not estimated)
         *
         **/
        bool isFixed() const;

        /** \brief Sets the state as fixed
         *
         * Sets the state as fixed
         *
         **/
        void fix();

        /** \brief Sets the state as estimated
         *
         * Sets the state as estimated
         *
         **/
        void unfix();

        /** \brief Prints all the elements of the state unit
		 *
		 * Prints all the elements of the state unit
		 *
		 **/
		virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
