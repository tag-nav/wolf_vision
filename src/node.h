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
         * Gets node ID. Inline function.
		 * 
         */
        unsigned int nodeId() const
        {
            return node_id_;
        };

        /** \brief Gets node label
         * 
         * Gets node label. Inline function.
		 * 
         */
        std::string nodeLabel() const
        {
            return label_;  
        };

        /** \brief Print node information
         * 
		 * Prints node information. Inine function.
         * \param _ntabs number of tabulations to print at the left of the printed information
         * \param _ost output stream
         *
         * Overload this function in derived classes to adapt the printed output to each object's relevant info.
		 * 
         */
        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const
        {
            _ost << label_ << " " << node_id_ << std::endl;
        };

    protected:

        /** \brief prints a number of tabulations.
         *
         * Prints a number of tabulations, i.e., "\t". Inline function.
         * \param _ntabs number of tabulations to print
         * \param _ost output stream
		 * 
         */
        void printNTabs(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const
        {
                for (unsigned int i = 0; i < _ntabs; i++) _ost << "\t";
        }
};

#endif /* NODE_H_ */
