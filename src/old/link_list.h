
/*
 * child_of.h
 *
 *  Created on: 17/06/2014
 *      \author: jsola
 */

#ifndef LINK_LIST_H_
#define LINK_LIST_H_

#include <iostream>
#include <memory>
#include <list>
#include <map>

using namespace std;

/** \brief List of weak pointers to nodes
 * 
 * Implements a list of weak pointers to nodes of the data tree
 * 
 **/
template<class NodeType>
class LinkList 
{
    public:
        typedef weak_ptr<NodeType> NodeWeakPtr;
        typedef shared_ptr<NodeType> NodePtr;
        typedef list<NodeType> NodeList;
        typedef typename list<NodeWeakPtr>::iterator NodeIter;
    
    public:
        NodeList node_list_;
    
    public:
        ~LinkList(void)
        {
            //
        }

        void addLink(const NodePtr & _ptr) 
        {
            node_list_.push_back(_ptr);
        }
        
        void deleteLink(const NodeIter & _iter) 
        {
            node_list_.erase(_iter);
        }
        
        const NodePtr nodePtr(NodeIter _iter) const
        {
            NodePtr nptr = _iter->lock();
            if (!nptr)
            {
                cerr << __FILE__ << ":" << __LINE__ << " LinkList::nodePtr threw weak" << endl;
                throw "WEAK";
            }
            return nptr;
        }

        void display(ostream& os = cout) const 
        {
            os << "NODE LIST [ ";
            for (auto iter = node_list_.begin(); iter != node_list_.end(); ++iter) 
            {
                    //os << iter->get() << " ";
            }
            os << " ]";
        }
};
#endif
