/**
 * \file branch_node_of.h
 *
 *  Created on: 22/06/2014
 *     \author: jsola
 */

#ifndef BRANCH_NODE_OF_H_
#define BRANCH_NODE_OF_H_

//eigen
#include <eigen3/Eigen/Dense>

//wolf
#include "parent_of.h"

using namespace Eigen;

/** \brief Parent of objects having expectation computations.
 *
 * Inherit from this class to be able to ask all the children for expectation errors.
 *
 * \param ChildType Type of Child. It corresponds to a particular typedef enum which specifies
 * all possible types of children. These enum's are usually defined at wolf.h.
 * \param Child the child class.
 *
 * This class implements the following functionalities:
 *  - Computing the size of all expectations of all children, and children of children, and so on.
 *  This must be used to build a storage vector of the correct size to hold all the results of expectation errors.
 *  - Querying all children, and children of children, down to the leafs of the tree, so that each leaf maps
 *  its expectation error into a correlative position of a unique storage vector.
 *  - Querying each child for its expectation error. This gets viral and propagates to each leaf,
 *  with the result of the storage vector getting updated with the expectation errors of all leafs.
 * 
 */
template<typename ChildType, class Child>
class BranchNodeOf : public ParentOf<ChildType, Child>
{
    protected:
        unsigned int expectation_error_size_; ///< Size of the expectation error vector

    public:
        /** \brief Constructor
         * 
         * Constructor.
         * 
         */
        BranchNodeOf() :
            expectation_error_size_(0)
        {
            //
        };        
        
        /** \brief Destructor
         * 
         * Destructor. Nothing to do.
         * 
         */
        virtual ~BranchNodeOf()
        {
            //
        };

        /** \brief Compute the size of the expectation error vector.
         * 
         * Compute the size of the expectation error vector by summing the sizes of all children.
         *
         * Note:
         *  - This function is viral and propagates downwards over the full tree.
         *  - Call this function before starting the computation of expectation errors. This way, the memory needed for
         *  all expectation errors of all children can be allocated beforehand.
         * 
         */
        virtual unsigned int computeExpectationErrorSize()
        {
            unsigned int size = 0;
            
            for (auto it = this->child_map_.begin(); it != this->child_map_.end(); ++it)
            {
                size += it->second->expectationErrorSize();
            }
            
            return size;
        }

        /** \brief Size of the expectation error
         *
         * Size of the expectation error. Call this function if you want to access the size
         * of the expectation error without having to compute it.
         *
         * Note:
         *  - Call computeExpectationErrorSize() beforehand if you want to be sure that it is up to date.
         *  - You can skip calling computeExpectationErrorSize() if you know that it is up to date or that it is zero (initial value).
         * 
         */
        virtual unsigned int expectationErrorSize()
        {
            if (expectation_error_size_ == 0)
            {
                expectation_error_size_ = computeExpectationErrorSize();
            }
            
            return expectation_error_size_;
        }

        /** \brief Create Eigen maps of the expectation errors.
         *
         * Create Eigen Maps of the expectation errors. Explore the tree downwards to the leafs,
         * and map each leaf's expectation error into a correlative segment of the storage vector provided.
         *
         * Note:
         *  - This function is viral and propagates downwards over the full tree.
         *  - Call computeExpectationErrorSize() at the top level of the tree, and
         *  - Resize the expectation error storage vector before calling this function.
         *  - Overload this function in derived classes in case you want non-standard behavior.
         *
         * \param _storage a reference to the storage vector of all expectation errors in the problem.
         * \param _idx a reference to an index to the start of the map to the expectation error storage.
         * This is an input-output parameter that advances each time a vector is mapped to the storage vector.
         * Its value should be zero at the time of calling this function at the top level of the tree.
         * 
         */
        virtual void mapExpectationErrors(VectorXs & _storage, unsigned int & _idx)
        {
            for (auto it = this->child_map_.begin(); it != this->child_map_.end(); ++it)
            {
                it->second->mapExpectationErrors(_storage, _idx);
            }
        }

        /** \brief Compute and aggregate all expectation errors of all children.
         *
         * Compute and aggregate all expectation errors of all children.
         *
         * Note:
         *  - This function is viral and propagates downwards over the full tree.
         *  - Call mapExpectationErrors() at the top level of the tree before calling this function.
         *  - Overload this function in derived classes in case you want non-standard behavior.
         * 
         */
        virtual void computeExpectationError()
        {
            for (auto it = this->child_map_.begin(); it != this->child_map_.end(); ++it)
            {
                it->second->computeExpectationError();
            }
        }
};
#endif /* BRANCH_NODE_OF_H_ */
