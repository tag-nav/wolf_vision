/**
 * \file parent_of.h
 *
 *  Created on: 17/06/2014
 *      \author: jsola
 */

#ifndef PARENT_OF_H_
#define PARENT_OF_H_

//std
#include <iostream>
#include <memory>
#include <map>

using namespace std;

/** \brief Parent relation.
 *
 * Inherit from this class to get a Parent relation to an object of another class.
 * The relation is through a set of lists to children objects (a multimap), categorized
 * by an enum key. The elements of this enum are defined elsewhere.
 *
 * The child class can be known only partially:
 * (i.e. class Child; class Parent : public ParentOf<Child> {...}; class Child { ... };)
 *
 * \param ChildType Type of Child. It corresponds to a particular typedef enum which specifies
 * all possible types of children. These enum's are usually defined at wolf.h.
 * \param Child the base class for children.
 *
 * See also ChildOf
 * 
 */
template<class ChildType, class Child>
class ParentOf
{
    public:
        typedef shared_ptr<Child> ChildPtr; ///< pointer to child
        typedef multimap<ChildType, ChildPtr> ChildMap; ///< multimap of pointers to children
        typedef typename multimap<ChildType, ChildPtr>::value_type ChildPair; ///< a ChildType:Child pair
        typedef typename multimap<ChildType, ChildPtr>::iterator ChildIter; ///< Iterator for the multimap.

    protected:
        ChildMap child_map_; ///< the categorized set of children

    public:
        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        ~ParentOf(void)
        {
            // cout << "ParentOf::dying" << endl;
        }

        /** \brief Add a child to the list of children
         * 
         * Add a child to the list of children
         * @param _key the child enum type
         * @param _ptr pointer to the child
         * 
         */
        ChildIter registerChild(const ChildType & _key, const ChildPtr & _ptr)
        {
            // cout << "ParentOf::register child in child_map_" << endl;
            return child_map_.insert(ChildPair(_key, _ptr));
        }
        
        /** \brief Remove a child from the list of children
         * 
         * Remove a child from the list of children
         * @param _iter an iterator to a particular child in the list.
         * 
         */
        void unregisterChild(const ChildIter & _iter)
        {
            // cout << "ParentOf::telling child to unlink from me" << endl;
            _iter->second->unlinkFromParent();
            // cout << "ParentOf::erasing child from my child_map_" << endl;
            child_map_.erase(_iter);
        }

        /**
         * \brief Access a reference to the multimap of children.
         */
        ChildMap & childMap(void)
        {
            return child_map_;
        }

        /**
         * \brief Access a constant reference to the multimap to children.
         */
        const ChildMap & childMap(void) const
        {
            return child_map_;
        }

        /** \brief Print some info
         * 
         * Print some info
         * @param os an ostream where to print info. Default is std::cout
         * 
         */
        void display(ostream& os = cout) const
        {
            os << "CHILD LIST [ ";
            for (auto iter = child_map_.begin(); iter != child_map_.end(); ++iter)
            {
                os << iter->first << " ";
            }
            os << " ]";
        }
};
#endif

