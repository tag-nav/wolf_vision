/**
 * \file node_constrainer.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef NODE_CONSTRAINER_H_
#define NODE_CONSTRAINER_H_

#include "node_linked.h"

/** \brief Node Constrainer of the Wolf Tree
 * \param UpperType the type of node one level up in the Wolf tree.
 * \param LowerType the type of node one level down in the Wolf tree.
 *
 * Node Constrainer of the Wolf Tree.
 *
 * The vector error_ is the target vector to minimize by an optimizer. It is stored remotely through an Eigen::Map
 *
 */
template<class UpperType, class LowerType>
class NodeConstrainer : public NodeLinked<UpperType, LowerType>
{

    protected:
        unsigned int dim_error_;
        Eigen::Map<Eigen::VectorXs> error_;

    protected:
        NodeConstrainer(
                const NodeLocation _loc, 
                const std::string& _label);

        NodeConstrainer( //
                const NodeLocation _loc, //
                const std::string& _label, //
                const typename NodeConstrainer<UpperType, LowerType>::UpNodeShPtr& _up_node_ptr);

        NodeConstrainer( //
                const NodeLocation _loc, //
                const std::string& _label, //
                const typename NodeConstrainer<UpperType, LowerType>::UpNodeShPtr& _up_node_ptr, //
                unsigned int _dim_error);

        virtual ~NodeConstrainer();

    public:
        /**
         * \brief Compute dim_error_ (recursive)
         *
         * Recursively traverses all the tree below the current node to compute the total error dim of the node.
         */
        unsigned int computeDimError();

        /** \brief Returns dim_error_
         *
         * Returns dim_error_
         *
         **/
        unsigned int dimError() const;

        /** \brief Remaps error_ vector recursively.
         *
         * Remaps error_ vector recursively. It will be called when a new node is added to down_node_list_ or recursively by an up node
         * \param _storage actual vector where data is stored
         * \param _idx index from which this map starts
         * \return the new index, pointing to the location next to the last mapped so far (this is to be input to the next remap() ).
         *
         **/
        unsigned int remap(Eigen::VectorXs& _storage, unsigned int _idx);

        /** \brief Remaps error_ vector recursively.
         *
         * Remaps error_ vector recursively. It will be called when a new node is added to down_node_list_ or recursively by an up node
         * \param _storage pointer to the actual vector where data is stored
         * \param _idx index, after the pointer, from which map starts
         * \return the new index, pointing to the location next to the last mapped so far (this is to be input to the next remap() ).
         *
         **/
        unsigned int remap(WolfScalar * _storage, unsigned int _idx);

        /** \brief Compute the error of the node (recursive)
         *
         * Calls computeError() recursively to down nodes, until reaching \a BOTTOM node,
         * where computeError() is overloaded with the actual computation of its error.
         *
         *   - For nodes \a TOP and \a MID, this method recursively calls computeError() of down nodes.
         *   - For nodes placed at the \a BOTTOM of the tree, this method needs to be overloaded (see e.g.
         *     CorrespondenceBase::computeError()).
         *
         * The result of this call is placed at the member vector %error_,
         * which is an Eigen::Map to an external, contiguous, error_storage vector.
         *
         **/
        virtual void computeError();

        /** \brief Compute the error of the node (recursive)
         *
         * Calls computeError() recursively to down nodes, until reaching \a BOTTOM node,
         * where computeError() is overloaded with the actual computation of its error.
         *
         *   - For nodes \a TOP and \a MID, this method recursively calls computeError() of down nodes.
         *   - For nodes placed at the \a BOTTOM of the tree, this method needs to be overloaded (see e.g.
         *     CorrespondenceBase::computeError()).
         *
         * The result of this call is placed at the provided vector \a _residuals, starting at position \a _idx and ranging
         * as far as the dimension of the error for the node. The index \a _idx in incremented by this function to serve
         * as the new index for the following call.
         **/
        virtual void computeError(Eigen::VectorXs & _residuals, unsigned int & _idx);

        /** \brief Compute the error of the node (recursive)
         *
         * Calls computeError() recursively to down nodes, until reaching \a BOTTOM node,
         * where computeError() is overloaded with the actual computation of its error.
         *
         *   - For nodes \a TOP and \a MID, this method recursively calls computeError() of down nodes.
         *   - For nodes placed at the \a BOTTOM of the tree, this method needs to be overloaded (see e.g.
         *     CorrespondenceBase::computeError()).
         *
         * The result of this call is placed at the provided Map<VectorXs> \a _residuals, starting at position \a _idx and ranging
         * as far as the dimension of the error for the node. The index \a _idx in incremented by this function to serve
         * as the new index for the following call.
         **/
        virtual void computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx);

        /** \brief Compute the error of the node (recursive)
         *
         * Calls computeError() recursively to down nodes, until reaching \a BOTTOM node,
         * where computeError() is overloaded with the actual computation of its error.
         *
         *   - For nodes \a TOP and \a MID, this method recursively calls computeError() of down nodes.
         *   - For nodes placed at the \a BOTTOM of the tree, this method needs to be overloaded (see e.g.
         *     CorrespondenceBase::computeError()).
         *
         * The result of this call is placed at the provided pointer \a _residuals plus \a _idx, and ranging
         * as far as the dimension of the error for the node. The index \a _idx is incremented by this function to serve
         * as the new index for the following call.
         **/
        virtual void computeError(WolfScalar * _residuals, unsigned int & _idx);

        /** \brief Gets a reference to the error of the node.
         * 
         * Gets a reference to the error of the node.
         * 
         */
        const virtual Eigen::Map<Eigen::VectorXs>& error() const;

        /** \brief pre-compute constants affecting the current node.
         *
         * This function pre-computes a number of constants to be reused by all the children of the node.
         * Usually, this pre-computation must be done each time the state vector changes, that is,
         * at each iteration of the optimizer.
         * 
         */
        virtual void precomputeConstants();

        /** \brief Prints tabulated information about this node.
         *
         * Prints information about this node. It adds a number of tabs given by _ntabs.
         *\param _ntabs the number of tabs.
         *\param _ost the stream it prints to
         * 
         */        
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;


};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

#include "wolf.h"
#include "node_terminus.h"

template<class UpperType, class LowerType>
NodeConstrainer<UpperType, LowerType>::NodeConstrainer( //
        const NodeLocation _loc, //
        const std::string& _label, //
        const typename NodeConstrainer<UpperType, LowerType>::UpNodeShPtr& _up_node_ptr, //
        unsigned int _dim_error) : //
            NodeLinked<UpperType, LowerType>(_loc, _label, _up_node_ptr), //
            dim_error_(_dim_error), //
            error_(nullptr, 0)
{
}

template<class UpperType, class LowerType>
NodeConstrainer<UpperType, LowerType>::NodeConstrainer( //
        const NodeLocation _loc, //
        const std::string& _label, //
        const typename NodeConstrainer<UpperType, LowerType>::UpNodeShPtr& _up_node_ptr) : //
            NodeLinked<UpperType, LowerType>(_loc, _label, _up_node_ptr), //
            dim_error_(0), //
            error_(nullptr, 0)
{
}

template<class UpperType, class LowerType>
NodeConstrainer<UpperType, LowerType>::NodeConstrainer(
        const NodeLocation _loc, 
        const std::string& _label) :
            NodeLinked<UpperType, LowerType>(_loc, _label), //
            dim_error_(0), //
            error_(nullptr, 0)
{
    //
}

template<class UpperType, class LowerType>
NodeConstrainer<UpperType, LowerType>::~NodeConstrainer()
{
    //
}

template<class UpperType, class LowerType>
unsigned int NodeConstrainer<UpperType, LowerType>::computeDimError()
{
    if (!this->isBottom())
    {
        dim_error_ = 0;
        for (auto const & down_node_shptr : this->downNodeList() )
        {
            down_node_shptr->computeDimError();
            dim_error_ += down_node_shptr->dimError();
        }
    }
    // else: do nothing, dim_error_ is already set for classes BOTTOM. TODO: How is this point assured ?? Do we have to add an assert at the constructor ??
    return dim_error_;
}

template<class UpperType, class LowerType>
inline unsigned int NodeConstrainer<UpperType, LowerType>::dimError() const
{
    return dim_error_;
}

template<class UpperType, class LowerType>
unsigned int NodeConstrainer<UpperType, LowerType>::remap(Eigen::VectorXs& _storage, unsigned int _idx)
{
    // always remap this node, regardless of lower nodes
    new (&error_) Eigen::Map<Eigen::VectorXs>(&_storage(_idx), dim_error_);

    if (!this->isBottom())
    {
        for (auto const & down_node_shptr : this->downNodeList() )
        {
            // delegate remap to down nodes
            _idx = down_node_shptr->remap(_storage, _idx); //forwards remap() to down nodes
        }
    }
    else //BOTTOM node case
    {
        _idx = _idx + dim_error_; // Just advance index according to error dimension
    }
    return _idx;
}

template<class UpperType, class LowerType>
unsigned int NodeConstrainer<UpperType, LowerType>::remap(WolfScalar * _storage, unsigned int _idx)
{
    // always remap this node, regardless of lower nodes
    new (&error_) Eigen::Map<Eigen::VectorXs>(_storage + _idx, dim_error_);

    if (!this->isBottom())
    {
        for (auto const & down_node_shptr : this->downNodeList() )
        {
            // delegate remap to down nodes
            _idx = down_node_shptr->remap(_storage, _idx); //forwards remap() to down nodes
        }
    }
    else
    {
        // Just advance index according to error dimension
        _idx = _idx + dim_error_;
    }
    return _idx;
}


template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::computeError()
{
    for (auto const & down_node_shptr : this->downNodeList() )
    {
        down_node_shptr->computeError(); //forwards error() to down nodes
    }
}

template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::computeError(Eigen::VectorXs & _residuals, unsigned int & _idx)
{
    for (auto const & down_node_shptr : this->downNodeList() )
    {
        down_node_shptr->computeError(_residuals, _idx);
    }
}


template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::computeError(Eigen::Map<Eigen::VectorXs> & _residuals, unsigned int & _idx)
{
    for (auto const & down_node_shptr : this->downNodeList() )
    {
        down_node_shptr->computeError(_residuals, _idx);
    }
}


template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::computeError(WolfScalar * _residuals, unsigned int & _idx)
{
    for (auto const & down_node_shptr : this->downNodeList() )
    {
        down_node_shptr->computeError(_residuals, _idx);
    }
}



template<class UpperType, class LowerType>
inline const Eigen::Map<Eigen::VectorXs>& NodeConstrainer<UpperType, LowerType>::error() const
{
    return error_;
}

template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::precomputeConstants()
{
        for (auto const & down_node_shptr : this->downNodeList() )
    {
        down_node_shptr->precomputeConstants();
    }
}

template<class UpperType, class LowerType>
inline void NodeConstrainer<UpperType, LowerType>::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeLinked<UpperType, LowerType>::printSelf(_ntabs, _ost);
    Node::printNTabs(_ntabs);
    _ost << "\tError      : ( " << error().transpose() << " )" << std::endl;
}

#endif /* NODE_CONSTRAINER_H_ */
