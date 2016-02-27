
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

// Fwd references
class LocalParametrizationBase;

//Wolf includes
#include "wolf.h"

//std includes
#include <iostream>

//class StateBlock
class StateBlock
{
    protected:
        //StateType type_;
        Eigen::VectorXs state_;
        bool fixed_;
        LocalParametrizationBase* local_param_ptr_;
        
    public:
        /** \brief Constructor with scalar pointer
         * 
         * Constructor with scalar pointer
         * \param _state is state vector
         * \param _type parametrization of the state
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * 
         **/
        StateBlock(const Eigen::VectorXs _state, bool _fixed = false);
        
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

//        /** \brief Returns the parametrization of the state
//         *
//         * Returns the parametrizationType (see wolf.h) of the state
//         *
//         **/
//        virtual StateType getType() const;

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

        bool hasLocalParametrization();

        LocalParametrizationBase* getLocalParametrizationPtr();

        /** \brief Prints all the elements of the state unit
         *
         * Prints all the elements of the state unit
         *
         **/
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
