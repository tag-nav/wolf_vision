#ifndef NODE_BASE_H_
#define NODE_BASE_H_

// Fwd references
namespace wolf{
class Problem;
}

// Wolf includes
#include "wolf.h"

// std includes


namespace wolf {

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree.
 * It implements the ID factory.
 *
 **/
class NodeBase
{
    private:
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)

    protected:
        unsigned int node_id_; ///< Node id. It is unique over the whole Wolf Tree
        std::string class_; ///< Text label identifying the class of node ("SENSOR", "FEATURE", etc)
        std::string type_; ///< Text label identifying the type within the class of node ("CAMERA", "POINT", etc)
        std::string name_; ///< Text label identifying each specific object ("left camera", "LIDAR 1", "PointGrey", "Andrew", etc)
        bool verbose_; 

    public: 

        /** \brief Constructor from label.
         *
         * Constructor from label
		 * 
         */
        NodeBase(std::string _class, bool _verbose = false);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~NodeBase();

        unsigned int nodeId() const;
        std::string getClass() const;
        std::string getName() const;
        void setName(const std::string& _name);
};

} // namespace wolf

#include <iostream>

namespace wolf{

inline NodeBase::NodeBase(std::string _class, bool _verbose) :
        node_id_(++node_id_count_), class_(_class), verbose_(_verbose)
{
    if (verbose_)
        std::cout << "NodeBase::NodeBase(). Id: " << node_id_ << " Label: " << class_ << std::endl;
}

inline NodeBase::~NodeBase()
{
    //
}

inline unsigned int NodeBase::nodeId() const
{
    return node_id_;
}

inline std::string NodeBase::getClass() const
{
    return class_;
}

inline std::string NodeBase::getName() const
{
    return name_;
}

inline void NodeBase::setName(const std::string& _name)
{
    name_ = _name;
}

} // namespace wolf

#endif /* NODE_BASE_H_ */
