#include "node_base.h"
#include "wolf_problem.h"

#include <iostream>


//init static node counter
unsigned int NodeBase::node_id_count_ = 0;

NodeBase::NodeBase(std::string _label, bool _verbose) :
        label_(_label), node_id_(++node_id_count_),
        //node_pending_(ADD_PENDING),
        verbose_(_verbose)
{
    if (verbose_)
        std::cout << "NodeBase::NodeBase(). Id: " << node_id_ << " Label: " << label_ << std::endl;
}

NodeBase::~NodeBase()
{
	//std::cout << "deleting NodeBase " << nodeId() << std::endl;
}

