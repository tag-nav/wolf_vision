/*
 * GraphNodeBase.h
 *
 *  Created on: June 17, 2014
 *      \author: andreu
 */

/**
 * \file graph_node_base.h
 */

#ifndef GraphNodeBase_H_
#define GraphNodeBase_H_

//std
#include <iostream>
#include <memory>

//wolf
#include "wolf.h"

using namespace std;
using namespace Eigen;

/** \brief Base class for all nodes in the graph
 * 
 * This class is the base for all nodes in the graph.
 * All nodes placed in the graph, from frames to features, should inherit from this class. 
 * It has three pointers:
 *    - to parent node
 *    - to a past correspondence node
 *    - to a future correspondence node
 * 
 */
class GraphNodeBase
{
    protected:
        /** \brief Pointer to parent node
         * 
         * Pointer to parent node
         * 
         **/
        weak_ptr<GraphNodeBase> parent_;
        
        /** \brief Pointer to future correspondence node
         * 
         * Pointer to future correspondence node, 
         * Therefore, this node [is | belongs to] a frame older than the pointed correspondence
         * 
         **/        
        weak_ptr<GraphNodeBase> forward_correspondence_; 
        
        /** \brief Pointer to past correspondence node
         * 
         * Pointer to past correspondence node, 
         * Therefore, this node [is | belongs to] a frame newer than the pointed correspondence
         * 
         **/        
        weak_ptr<GraphNodeBase> backward_correspondence_;         
        
        //debugging
        unsigned int id_; 

    public:

        /** \brief Constructor
         * 
         * Constructor 
         * \param _parent_node pointer to parent node
         * \param _id just for debugging
         * 
         */
        GraphNodeBase(const shared_ptr<GraphNodeBase>&  _parent_node, const int _id);

        /** \brief Destructor
         * 
         * Destructor. Tree will be deleted from down to top, so pointers will be deleted first than parent objects. 
         * 
         */
        virtual ~GraphNodeBase();
        
        /** \brief Sets forward_correspondence_ pointer
         * 
         * Sets forward_correspondence_ pointer
         * \param _bckwd_c pointer to backward correspondence node
         * 
         */        
        //setForwardCorrespondence(shared_ptr<GraphNodeBase> & _fwd_c);
        void setForwardCorrespondence(GraphNodeBase & _fwd_c);
        
        /** \brief Sets backward_correspondence_ pointer
         * 
         * Sets backward_correspondence_ pointer
         * \param _bckwd_c pointer to backward correspondence node
         * 
         */        
        //setBackwardCorrespondence(shared_ptr<GraphNodeBase> & _bckwd_c);
        void setBackwardCorrespondence(GraphNodeBase & _bckwd_c);
        
        //debugging
        void print();
        
};
#endif /* GraphNodeBase_H_ */
