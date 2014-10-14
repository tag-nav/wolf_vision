/**
 * \file node.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef NODE_H_
#define NODE_H_

#include <iostream>

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree.
 * It implements the ID factory.
 *
 **/
class Node
{
    private:
        std::string label_; ///< Text label identifying the node
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)
        unsigned int node_id_; ///< Node id. It is unique over the whole Wolf Tree

    protected:

        /** \brief Constructor from label.
         *
         * Constructor from label
		 * 
         */
        Node(std::string _label);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~Node();

    public:
        /** \brief Gets node ID
         *
         * Gets node ID
		 * 
         */
        unsigned int nodeId() const;

        /** \brief Gets node label
         * 
         * Gets node label
		 * 
         */
        std::string nodeLabel() const;

        /** \brief Print node information
         * 
		 * Prints node information.
         * \param _ntabs number of tabulations to print at the left of the printed information
         * \param _ost output stream
         *
         * Overload this function in derived classes to adapt the printed output to each object's relevant info.
		 * 
         */
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

    protected:

        /** \brief prints a number of tabulations.
         *
         * Prints a number of tabulations, i.e., "\t".
         * \param _ntabs number of tabulations to print
         * \param _ost output stream
		 * 
         */
        void printNTabs(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};

//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////

Node::Node(std::string _label) :
        label_(_label), //
        node_id_(++node_id_count_)
{
    //    std::cout << "NodeID::constructor. Id: " << node_id_ << std::endl;
}

Node::~Node()
{
    //    std::cout << "NodeID::destructor. Id: " << node_id_ << std::endl;
}

inline unsigned int Node::nodeId() const
{
    return node_id_;
}

inline std::string Node::nodeLabel() const
{
    return label_;
}

void Node::print(unsigned int _ntabs, std::ostream& _ost) const
{
    _ost << label_ << " " << node_id_ << std::endl;
}

inline void Node::printNTabs(unsigned int _ntabs, std::ostream& _ost) const
{
    for (unsigned int i = 0; i < _ntabs; i++)
        _ost << "\t";
}

#endif /* NODE_H_ */
