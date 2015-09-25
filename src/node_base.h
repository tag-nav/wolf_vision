#ifndef NODE_BASE_H_
#define NODE_BASE_H_

#include <iostream>
#include "wolf.h"

class WolfProblem;

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree.
 * It implements the ID factory.
 *
 **/
class NodeBase
{
    protected:
        std::string label_; ///< Text label identifying the node
        unsigned int node_id_; ///< Node id. It is unique over the whole Wolf Tree
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)
        PendingStatus node_pending_; ///< Pending to be added/updated in the filter or optimizer
        bool verbose_; 

    public: 

        /** \brief Constructor from label.
         *
         * Constructor from label
		 * 
         */
        NodeBase(std::string _label, bool _verbose = false);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~NodeBase();

        /** \brief Gets node ID
         *
         * Gets node ID. Inline function.
		 * 
         */
        unsigned int nodeId() const;

        /** \brief Gets node label
         * 
         * Gets node label. Inline function.
		 * 
         */
        std::string nodeLabel() const;

        /** \brief Gets the node pending status (pending or not to be added/updated in the filter or optimizer)
         *
         * Gets the node pending status (pending or not to be added/updated in the filter or optimizer)
		 *
         */
        PendingStatus getPendingStatus() const;

        /** \brief Sets the node pending status (pending or not to be added/updated in the filter or optimizer)
         *
         * Sets the node pending status (pending or not to be added/updated in the filter or optimizer)
		 *
         */
        void setPendingStatus(PendingStatus _pending);

        /** \brief Print node information
         * 
		 * Prints node information. Inine function.
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
         * Prints a number of tabulations, i.e., "\t". Inline function.
         * \param _ntabs number of tabulations to print
         * \param _ost output stream
		 * 
         */
        void printTabs(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;
};

#endif /* NODE_BASE_H_ */
