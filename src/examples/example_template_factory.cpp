/*
 * \file test_template_factory.cpp
 *
 *  Created on: Mar 11, 2016
 *      \author: jsola
 */

// includes for the factory design
#include <stdexcept>
#include <typeinfo>

// general includes
#include <iostream>

// In their respective object .h and .cpp files
// things constrained == anything, deriving from a base type
//  type base
class Object
{
};
// type A
class ObjectA : public Object
{
};
// type B
class ObjectB : public Object
{
};
// type C
class ObjectC : public Object
{
};

//----------------------------------------------------------------------
// In constraint_base.h
class Constraint
{
    public:
        virtual ~Constraint()
        {
        }
        virtual void print()
        {
            std::cout << "constraint base" << std::endl;
        }
};
// Default factory throws
template<typename SelfType, typename OtherType>
class ConstraintFactory
{
    public:
        static Constraint* newConstraint()
        {
            throw std::invalid_argument("Unknown Constraint between types "); // << typeid(SelfType).name() << " and " << typeid(SelfType).name() << "!");
        }
};

//----------------------------------------------------------------------
// in c_a_b.h
// constraint A-B
class ConstraintAB : public Constraint
{
        virtual void print()
        {
            std::cout << "constraint A-B" << std::endl;
        }
};
template<>
class ConstraintFactory<ObjectA, ObjectB>
{
    public:
        static Constraint* newConstraint()
        {
            return new ConstraintAB;
        }
};

//----------------------------------------------------------------------
// In c_a_c.h
// constraint A-C
class ConstraintAC : public Constraint
{
        virtual void print()
        {
            std::cout << "constraint A-C" << std::endl;
        }
};
template<>
class ConstraintFactory<ObjectA, ObjectC>
{
    public:
        static Constraint* newConstraint()
        {
            return new ConstraintAC;
        }
};

//----------------------------------------------------------------------
// In c_b_c.h
// constraint B-C
class ConstraintBC : public Constraint
{
        virtual void print()
        {
            std::cout << "constraint B-C" << std::endl;
        }
};
template<>
class ConstraintFactory<ObjectB, ObjectC>
{
    public:
        static Constraint* newConstraint()
        {
            return new ConstraintBC;
        }
};

//----------------------------------------------------------------------
// In p_base.h
class Processor
{
    public:
        virtual ~Processor()
        {
        }
        ;
        template<typename SelfType, typename OtherType>
        void process(SelfType* _self_ptr, OtherType* _other_ptr)
        {
            Constraint* c_ptr = ConstraintFactory<SelfType, OtherType>::newConstraint();
            c_ptr->print();
        }
};

//----------------------------------------------------------------------
// In p_a_b.h : has no derived pointers as members
class ProcessorAB : public Processor
{
    public:
        virtual ~ProcessorAB()
        {
        }
};

//----------------------------------------------------------------------
// In p_a_c.h : has one derived pointer ads member
class ProcessorAC : public Processor
{
    public:
        ObjectA* ta_;
        ProcessorAC(Object* _ta) :
                ta_((ObjectA*)_ta)
        {
        }
        void process(Object* _tc)
        {
            Processor::process(ta_, (ObjectC*)_tc); // cast needed
        }
        virtual ~ProcessorAC()
        {
        }
};

//----------------------------------------------------------------------
// In p_b_c.h : has both derived pointers as members
class ProcessorBC : public Processor
{
    public:
        ObjectB* tb_;
        ObjectC* tc_;
        ProcessorBC(Object* _tb, Object* _tc) :
                tb_((ObjectB*)_tb), tc_((ObjectC*)_tc)
        {
        }
        void process()
        {
            Processor::process(tb_, tc_);
        }
        virtual ~ProcessorBC()
        {
        }
};

//----------------------------------------------------------------------
// In application.cpp
int main()
{
    Object* a_p = new ObjectA;
    Object* b_p = new ObjectB;
    Object* c_p = new ObjectC;

    ProcessorAB pab;
    ProcessorAC pac(a_p);
    ProcessorBC pbc(b_p, c_p);

    Processor* pab_p = new ProcessorAB;
    Processor* pac_p = new ProcessorAC(a_p);
    Processor* pbc_p = new ProcessorBC(b_p, c_p);

    std::cout << "object processors:" << std::endl;
    pab.process((ObjectA*)a_p, (ObjectB*)b_p); // This use base process() --> need to cast when calling
    pac.process(c_p); // This uses overloaded process() --> casted by the class
    pbc.process(); // This uses overloaded process() --> casted by the class

    std::cout << "\npointers to processors:" << std::endl;
    pab_p->process((ObjectA*)a_p, (ObjectB*)b_p); // This uses base process() --> need to cast when calling
    pac_p->process((ObjectA*)a_p, (ObjectC*)c_p);
    pbc_p->process((ObjectB*)b_p, (ObjectC*)c_p);

    std::cout << "\ncall with base pointers->throw:" << std::endl;
    pab.process(a_p,b_p);

    return 0;
}
