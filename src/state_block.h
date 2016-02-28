
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

//std includes
#include <iostream>
#include <vector>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "local_parametrization_base.h"

/** \brief class StateBlock
 *
 * This class implements a state block for general nonlinear optimization. It offers the following functionality:
 *  - A state vector storing the state values.
 *  - A key to indicate whether the state is fixed or not.
 *     - Fixed state blocks are not estimated and treated by the estimator as fixed parameters.
 *     - Non-fixed state blocks are estimated.
 *  - A local parametrization useful for optimizing in the tangent space to the manifold.
 */
class StateBlock
{
    protected:
        Eigen::VectorXs state_; ///< State vector storing the state values
        bool fixed_; ///< Key to indicate whether the state is fixed or not
        LocalParametrizationBase* local_param_ptr_; ///< Local parametrization useful for optimizing in the tangent space to the manifold
        
    public:
        /** \brief Constructor with scalar pointer
         * 
         * \param _state is state vector
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         **/
        StateBlock(const Eigen::VectorXs _state, bool _fixed = false);
        
        /** \brief Destructor
         **/
        virtual ~StateBlock();

        /** \brief Returns the pointer to the first element of the state
         **/
        WolfScalar* getPtr();
        
        /** \brief Returns the state vector
         **/
        Eigen::VectorXs getVector();

        /** \brief Sets the state vector
         **/
        void setVector(const Eigen::VectorXs& _state);

        /** \brief Returns the state size
         **/
        unsigned int getSize() const;

        /** \brief Returns if the state is fixed (not estimated)
         **/
        bool isFixed() const;

        /** \brief Sets the state as fixed
         **/
        void fix();

        /** \brief Sets the state as estimated
         **/
        void unfix();

        bool hasLocalParametrization();

        LocalParametrizationBase* getLocalParametrizationPtr();

        /** \brief Prints all the elements of the state unit
         **/
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};
#endif
