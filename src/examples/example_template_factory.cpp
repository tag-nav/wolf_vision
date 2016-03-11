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

// things constrained == anything, deriving from a base type
class Object{}; //  type base
class ObjectA:public Object{}; // type A
class ObjectB:public Object{}; // type B
class ObjectC:public Object{}; // type C

// c_base.h
class Constraint
{
    public:
        virtual ~Constraint(){}
        virtual void print()
        {
            std::cout << "constraint base" << std::endl;
        }
};

// factory.h
template<typename SelfType, typename OtherType>
class ConstraintFactory{
    public:
        static Constraint* newConstraint()
        {
            throw std::invalid_argument("Unknown Constraint between types ");// << typeid(SelfType).name() << " and " << typeid(SelfType).name() << "!");
        }
};

// c_a_b.h
class ConstraintAB : public Constraint{virtual void print(){std::cout << "constraint A-B" << std::endl;}}; // constraint A-B

template<>
class ConstraintFactory<ObjectA, ObjectB>{
    public:
        static Constraint* newConstraint(){
            return new ConstraintAB;
        }
};

// c_a_c.h
class ConstraintAC : public Constraint{virtual void print(){std::cout << "constraint A-C" << std::endl;}}; // constraint A-C

template<>
class ConstraintFactory<ObjectA, ObjectC>{
    public:
       static Constraint* newConstraint(){
            return new ConstraintAC;
        }
};

// c_b_c.h
class ConstraintBC : public Constraint{virtual void print(){std::cout << "constraint B-C" << std::endl;}}; // constraint B-C

template<>
class ConstraintFactory<ObjectB, ObjectC>{
    public:
      static Constraint* newConstraint(){
            return new ConstraintBC;
        }
};

// p_base.h
class Processor
{
    public:
        virtual ~Processor(){};
        template<typename SelfType, typename OtherType>
        void process(SelfType* _self_ptr, OtherType* _other_ptr){
            Constraint* c_ptr = ConstraintFactory<SelfType,OtherType>::newConstraint();
            c_ptr->print();
        }
};

// p_a_b.h : has no derived pointers as members
class ProcessorAB : public Processor{
    public:
        virtual ~ProcessorAB(){};
};

// p_a_c.h : has one derived pointer ads member
class ProcessorAC : public Processor{
    public:
        ObjectA* ta_;
        ProcessorAC(Object* _ta):ta_((ObjectA*)_ta){}
        void process(Object* _tc){Processor::process(ta_,(ObjectC*)_tc);}
        virtual ~ProcessorAC(){};
};

// p_b_c.h : has both derived pointers as members
class ProcessorBC : public Processor{
    public:
        ObjectB* tb_;
        ObjectC* tc_;
        ProcessorBC(Object* _tb, Object* _tc):tb_((ObjectB*)_tb),tc_((ObjectC*)_tc){}
        void process(){Processor::process(tb_,tc_);}
        virtual ~ProcessorBC(){};
};

int main()
{
    Object* a = new ObjectA;
    Object* b = new ObjectB;
    Object* c = new ObjectC;

    ProcessorAB pab;
    ProcessorAC pac(a);
    ProcessorBC pbc(b,c);

    Processor* pab_p = new ProcessorAB;
    Processor* pac_p = new ProcessorAC(a);
    Processor* pbc_p = new ProcessorBC(b,c);

    std::cout << "object processors:" << std::endl;
    pab.process((ObjectA*)a,(ObjectB*)b); // This use base process() --> need to cast when calling
    pac.process(c); // This uses overloaded process() --> casted by the class
    pbc.process(); // This uses overloaded process() --> casted by the class

    std::cout << "\npointers to processors:" << std::endl;
    pab_p->process((ObjectA*)a,(ObjectB*)b); // This use base process() --> need to cast when calling
    pac_p->process((ObjectA*)a,(ObjectC*)c);
    pbc_p->process((ObjectB*)b,(ObjectC*)c);

//    pab.process(tb,ta);

    return 0;
}
