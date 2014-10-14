/**
 * \file leaf_node_of.h
 *
 *  Created on: 25/06/2014
 *     \author: jsola
 */

#ifndef LEAF_NODE_OF_H_
#define LEAF_NODE_OF_H_

#include "child_of.h"

using namespace Eigen;

/**
 * \brief Base class for leaf nodes in the Wolf tree structure.
 *
 * This class implements common functionality for leaf objects in the Wolf structure:
 *  - Return the own expectation error size.
 *  - Map the own expectation errors onto an external storage vector.
 *  - Compute the own expectation error.
 *
 * Inherit from this class when implementing derived classes intended to be the last leaves in the Wolf tree.
 *
 * \param Parent the class of the parent node in the Wolf tree.
 */
template<class Parent>
class LeafNodeOf : public ChildOf<Parent>
{
    protected:
        VectorXs expectation_error_;            ///< Self expectation error
        Map<VectorXs> expectation_error_map_;   ///< Mapped expectation error

    public:
        /** \brief Constructor for local storage.
         * 
         * Constructor for local storage:
         * \param _pptr pointer to parent
         * \param _size size of expectation error
         * 
         */
        LeafNodeOf(const shared_ptr<Parent> & _pptr, const unsigned int _size) :
            ChildOf<Parent>(_pptr), //
            expectation_error_(_size), //
            expectation_error_map_(expectation_error_.data(), _size) //
        {
            //
        }

        /** \brief Constructor for remote storage.
         * 
         * Constructor for local storage:
         * \param _pptr pointer to parent
         * \param _st_remote remote vector to allocate storage
         * \param _idx index to remote vector from which start the map
         * \param _size size of expectation error
         * 
         */
        LeafNodeOf(const shared_ptr<Parent> & _pptr, VectorXs& _st_remote, const unsigned int _idx, const unsigned int _size) :
            ChildOf<Parent>(_pptr), //
            expectation_error_map_(
                ((_idx + _size <= _st_remote.size()) ? _st_remote.data() + _idx : NULL), //
                ((_idx + _size <= _st_remote.size()) ? _size : 0))
        {
            //
        }        
        
        virtual ~LeafNodeOf(){};

        /**
         * \brief Expectation error size
         */
        virtual unsigned int expectationErrorSize()
        {
            return expectation_error_map_.size();
        };

        /**
         * \brief Map expectation errors to external storage vector.
         */
        virtual void mapExpectationErrors(VectorXs & _storage, unsigned int & _idx)
        {
            // In-place new of the existing map.
            new (&expectation_error_map_) Map<VectorXs>(&_storage(_idx), expectationErrorSize());

            // increment the pointer with the current error size.
            _idx += expectationErrorSize();
        }

        /**
         * \brief Force compute expectation error.
         *
         * Force derived classes to implement the computation of the expectation error.
         */
        virtual VectorXs computeExpectationError() = 0;

};

#endif /* LEAF_NODE_OF_H_ */
