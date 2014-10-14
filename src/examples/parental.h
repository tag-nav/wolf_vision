/*
 * \file parental.h
 *
 *  Created on: 16/06/2014
 *      \author: jsola
 */

#ifndef PARENTAL_H_
#define PARENTAL_H_

#include <iostream>
#include <tr1/memory>
#include <map>

using namespace std;


class Person
{
    public:
        typedef enum
        {
            UNDEFINED, MALE, FEMALE
        } Gender;
        Gender gender_;
        string name_;
        /** This is implemented here, at the very base class Person.
         *
         */
        virtual string name();
        /** This will be implemented only at the bottom of the derivations,
         * Father, Boy and Girl
         */
        virtual string relative() = 0;

        virtual ~Person();

        /** This will be implemented at the base class of each relative,
         * Parent and Child.
         */
        virtual void process() = 0;
};

typedef shared_ptr<Person> PersonPtr;

class Child;

typedef shared_ptr<Child> ChildPtr;

class Parent : public Person
{
    public:
        multimap<Gender, ChildPtr> children_;

        virtual ~Parent();
        void newChild(ChildPtr & _child_ptr);
        virtual void process();

        /** This forces this class to be Abstract (i.e., having a pure virtual method). However,
         * this virtual function is inherited from Person and only implemented in Father.
         *
         * I have purposely omitted it in class Child to illustrate the difference:
         * Parent is Abstract, while Child is not -- but the implementations happen exactly at the
         * same level, i.e., Father and Boy / Girl, as shown here
         *  - Person
         *    - relative() = 0          // declared --> Person is abstract
         *    - Parent : Person
         *      - relative() = 0;       // declared --> Parent is abstract
         *      - Father : Parent
         *        - relative(){ xx }    // implemented
         *    - Child : Person
         *      - // inexistent relative() --> Child is regular
         *      - Boy : Child
         *        - relative(){ xx }    // implemented
         *      - Girl : Child
         *        - relative(){ xx }    // implemented
         *
         * So, what the fuck?
         */
        virtual string relative() = 0;

        typedef multimap<Gender, ChildPtr> ChildMap;
        typedef multimap<Gender, ChildPtr>::value_type ChildPair;
        typedef multimap<Gender, ChildPtr>::iterator ChildIter;

};



typedef shared_ptr<Parent> ParentPtr;

class Child : public Person
{
    public:
        ParentPtr father_ptr_;

        Child(ParentPtr & _father_ptr);
        virtual ~Child();
        virtual void process();
};

class Father : public Parent
{
    public:
        Father(string _name);
        string relative();
};

class Boy : public Child
{
    public:
        Boy(ParentPtr _father_ptr, string _name);
        string relative();
};

class Girl : public Child
{
    public:
        Girl(ParentPtr _father_ptr, string _name);
        string relative();
};




/*
 * Start definitions
 */

inline Person::~Person()
{
    cout << "Person dying" << endl;
}

inline string Person::name()
{
    return name_;
}

inline Parent::~Parent()
{
    cout << "Parent dying" << endl;
}

inline void Parent::process(){
    cout << "I am " << relative() << name_;
    cout << ", my children are: " << endl;
    for(auto it = children_.begin(); it != children_.end(); ++it )
    {
        cout << (it->first == Person::MALE ? "BOY" : "GIRL") << "\t";
        cout << it->second->name();
        cout << endl;
    }
    cout << "My boys are: " << endl;
    auto boys_it = children_.equal_range(MALE);
    for (auto it = boys_it.first; it != boys_it.second; ++it) {
        cout << "\t" << it->second->name() << endl;
    }
    cout << "My girls are: " << endl;
    auto girls_it = children_.equal_range(FEMALE);
    for (auto it = girls_it.first; it != girls_it.second; ++it) {
        cout << "\t" << it->second->name() << endl;
    }
}

inline void Parent::newChild(ChildPtr & _child_ptr)
{
    children_.insert(ChildPair(_child_ptr->gender_, _child_ptr));
}

inline Child::Child(ParentPtr & _father_ptr)
{
    father_ptr_ = _father_ptr;
}

inline Child::~Child()
{
    cout << "Child dying" << endl;
}

inline Father::Father(string _name) :
        Parent()
{
    gender_ = MALE;
    name_ = _name;
}


inline void Child::process()
{
    cout << "I am " << name_ << ", ";
    cout << relative();
    cout << " of " << father_ptr_->name();
    cout << endl;
}

inline Boy::Boy(ParentPtr _father_ptr, string _name) :
        Child(_father_ptr)
{
    gender_ = MALE;
    name_ = _name;
}

inline Girl::Girl(ParentPtr _father_ptr, string _name) :
        Child(_father_ptr)
{
    gender_ = FEMALE;
    name_ = _name;
}

inline string Father::relative()
{
    return "father";
}

inline string Boy::relative()
{
    return "son";
}

inline string Girl::relative()
{
    return "daughter";
}

#endif /* TEST_WEAK_POINTED_MULTIMAP_H_ */
