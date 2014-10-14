/*
 * test_virtual_hierarchy.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: jsola
 */

#include <list>

class N
{
    private:
        unsigned int id_;
        static unsigned int id_count_;
    protected:
        N() : id_(++id_count_) { }
        virtual ~N() { }
    public:
        unsigned int id() { return id_; }
};

/*
 * Virtual inheritance solves the "diamond of death" problem.
 */
template<class U>
class Up : virtual public N
{
    private:
        U* up_ptr_;
    protected:
        Up(U* _up_ptr) : up_ptr_(_up_ptr) { }
        virtual ~Up() { }
        U* upPtr() { return up_ptr_; }
};

/*
 * Virtual inheritance solves the "diamond of death" problem.
 */
template<class D>
class Down : virtual public N
{
    private:
        std::list<D*> down_list_;
    protected:
        Down() { }
        virtual ~Down() { }
    public:
        void addToList(D* _down_ptr) { down_list_.push_back(_down_ptr); }
        std::list<D*> downList() { return down_list_; }
        virtual void print();
};

template<class S>
class Side : virtual public N
{
    private:
        std::list<S*> side_list_;
    protected:
        Side() { }
        virtual ~Side() { }
    public:
        void addToList(S* _side_ptr) { side_list_.push_back(_side_ptr); }
        std::list<S*> sideList() { return side_list_; }
};

class Bot : virtual public N
{
    protected:
        virtual ~Bot() { }
    public:
        virtual void print() { }
};

template<class D>
class Explorer : public Down<D>
{
    protected:
        Explorer() { }
        virtual ~Explorer() { }
    public:
        virtual void explore();
};

class Explored : public Bot
{
    protected:
        virtual ~Explored() { }
    public:
        virtual void explore() { }
};

// a bunch of fwd_decs
class VehNode;
class FrmNode;
class CapNode;
class FeaNode;
class CorNode;
class TrSNode;
class SenNode;

class Veh
{
    public:
        Veh() : v_(1){}
        void duplicate(){v_ *= 2;}
        double v(){return v_;}
    private:
        double v_;
};
class Frm
{
    public:
        Frm() : f_(1){}
        void duplicate(){f_ *= 2;}
        double f(){return f_;}
    private:
        double f_;
};
class Cap
{
    public:
        Cap() : c_(1){}
        void duplicate(){c_ *= 2;}
        double c(){return c_;}
    private:
        double c_;
};
class Fea
{
    public:
        Fea() : ft_(1){}
        void duplicate(){ft_ *= 2;}
        double ft(){return ft_;}
    private:
        double ft_;
};
class Sen
{
    public:
        Sen() : s_(1){}
        void duplicate(){s_ *= 2;}
        double s(){return s_;}
    private:
        double s_;
};

// Derived classes for all levels of the tree
class VehNode : public Veh, public Explorer<FrmNode>, public Down<SenNode>
{
    public:
        VehNode() { }
        virtual ~VehNode() { }
        void print(); // Overload because I am Top and have both Down and Explorer children.
};
class FrmNode : public Frm, public Up<VehNode>, public Explorer<CapNode>
{
    public:
        FrmNode(VehNode* _veh_ptr) : Up<VehNode>(_veh_ptr) { }
        virtual ~FrmNode() { }
};
class CapNode : public Cap, public Up<FrmNode>, public Explorer<FeaNode>, public Side<TrSNode>
{
    public:
        CapNode(FrmNode* _frm_ptr) : Up<FrmNode>(_frm_ptr) { }
        virtual ~CapNode() { }
        void explore(); // Overload because I have both Explorer and Side children
};
class FeaNode : public Fea, public Up<CapNode>, public Explorer<CorNode>
{
    public:
        FeaNode(CapNode* _cap_ptr) : Up<CapNode>(_cap_ptr) { }
        virtual ~FeaNode() { }
};
class CorNode : public Up<FeaNode>, public Explored
{
    public:
        CorNode(FeaNode* _fea_ptr) : Up<FeaNode>(_fea_ptr) { }
        virtual ~CorNode() { }
};
class TrSNode : public virtual N
{
    public:
        TrSNode() { }
        virtual ~TrSNode() { }
};
class SenNode : public Sen, public Up<VehNode>, public Bot
{
    public:
        SenNode(VehNode* _veh_ptr) : Up<VehNode>(_veh_ptr) { }
        virtual ~SenNode() { }
};

/////////////////////
// IMPLEMENTATIONS, here to avoid incomplete types and unwanted #includes
/////////////////////

#include <iostream>
using namespace std;

template<class D>
void Down<D>::print()
{
    cout << this->id() << ":( ";
    for (auto const & it_ptr : this->downList())
        cout << it_ptr->id() << " ";
    cout << ")" << endl;
    for (auto const & it_ptr : this->downList())
        it_ptr->print();
}

template<class D>
void Explorer<D>::explore()
{
    cout << this->id() << ":( "; // Yes I look sad but I'm OK.
    for (auto const & it_ptr : Down<D>::downList())
        cout << it_ptr->id() << " ";
    cout << ")" << endl;
    for (auto const & it_ptr : Down<D>::downList())
        it_ptr->explore();
}

void VehNode::print()
{
    cout << "Vehicle Own Field: v_ = " << v() << endl;
    cout << "Vehicle Hardware:" << endl;
    Down < SenNode > ::print();
    cout << "Vehicle Data:" << endl;
    Down < FrmNode > ::print();

}
void CapNode::explore()
{
    cout << this->id() << ":( "; // Yes I look sad but I'm OK.
    for (auto const & it_ptr : Explorer<FeaNode>::downList())
        cout << it_ptr->id() << " ";
    cout << "/ ";
    for (auto const & it_ptr : Side<TrSNode>::sideList())
        cout << it_ptr->id() << " ";
    cout << ")" << endl;
    for (auto const & it_ptr : Explorer<FeaNode>::downList())
        it_ptr->explore();
}

///////////////////////
// START APPLICATION
///////////////////////

unsigned int N::id_count_ = 0;

int main()
{
    // Create all nodes with up-pointers already set up
    VehNode V;
    SenNode S0(&V), S1(&V);
    FrmNode F0(&V), F1(&V);
    CapNode C00(&F0), C01(&F0), C10(&F1), C11(&F1);
    FeaNode f000(&C00), f001(&C00), f010(&C01), f011(&C01), f100(&C10), f101(&C10), f110(&C11), f111(&C11);
    TrSNode T0001, T0010, T0011, T0110, T0111, T1011;

    // Add sensors to vehicle
    V.Down < SenNode > ::addToList(&S0);
    V.Down < SenNode > ::addToList(&S1);

    // Add frames to vehicle
    V.Down < FrmNode > ::addToList(&F0);
    V.Down < FrmNode > ::addToList(&F1);

    // Add captures to frames
    F0.Down<CapNode>::addToList(&C00);
    F0.Down<CapNode>::addToList(&C01);
    F1.Down<CapNode>::addToList(&C10);
    F1.Down<CapNode>::addToList(&C11);

    // Add features to captures
    C00.Down<FeaNode>::addToList(&f000);
    C00.Down<FeaNode>::addToList(&f001);
    C01.Down<FeaNode>::addToList(&f010);
    C01.Down<FeaNode>::addToList(&f011);
    C10.Down<FeaNode>::addToList(&f100);
    C10.Down<FeaNode>::addToList(&f101);
    C11.Down<FeaNode>::addToList(&f110);
    C11.Down<FeaNode>::addToList(&f111);

    // Add trans-sensors to captures
    C00.Side<TrSNode>::addToList(&T0001);
    C00.Side<TrSNode>::addToList(&T0010);
    C00.Side<TrSNode>::addToList(&T0011);
    C01.Side<TrSNode>::addToList(&T0110);
    C01.Side<TrSNode>::addToList(&T0111);
    C10.Side<TrSNode>::addToList(&T1011);

    // explore() : means we are calling advanced functionality from Explorer classes. Here, we just fake.
    // print()   : means we just print linkage info.
    cout << "V.explore():" << endl;
    V.explore();
    V.duplicate();
    cout << "V.print():" << endl;
    V.print();

    return 0;
}
