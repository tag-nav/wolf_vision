/**
 * \file test_sh_ptr.cpp
 *
 *  Created on: Oct 4, 2016
 *      \author: jsola
 *
 *  Complete Wolf tree skeleton with smart pointers
 *
 */

// C++11
#include <memory>

using std::shared_ptr;
using std::weak_ptr;
using std::make_shared;
using std::enable_shared_from_this;

// std
#include <list>
#include <iostream>
using std::list;
using std::cout;
using std::endl;

namespace wolf
{
// fwds
class H; // hardware
class S; // sensor
class p; // processor
class T; // trajectory
class F; // frame
class C; // capture
class f; // feature
class c; // constraint
class M; // map
class L; // landmark

// all wolf tree is here
class P : public enable_shared_from_this<P>
{
        shared_ptr<H> H_ptr_;
        shared_ptr<T> T_ptr_;
        shared_ptr<M> M_ptr_;

    public:
        P();
        ~P(){cout << "destruct P" << endl;}
        void setup();
        shared_ptr<H> getH(){return H_ptr_;}
        shared_ptr<T> getT(){return T_ptr_;}
        shared_ptr<M> getM(){return M_ptr_;}
};

class H : public enable_shared_from_this<H>
{
        weak_ptr<P> P_ptr_;
        list<shared_ptr<S>> S_list_;
    public:
        H(){cout << "construct H" << endl;}
        ~H(){cout << "destruct H" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (P_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<S>>& getSlist() {return S_list_;}
        void addS(shared_ptr<S> _S);
};

class S : public enable_shared_from_this<S>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<H> H_ptr_;
        list<shared_ptr<p>> p_list_;

        list<weak_ptr<C>> C_list_; // List of all captures
    public:
        S(){cout << "construct S" << endl;}
        ~S(){cout << "destruct S" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
             if (!P_sh)
                 P_sh = getH()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<H> getH(){
            shared_ptr<H> H_sh = H_ptr_.lock();
            // if (H_sh) // check will be made by caller after all
            return H_sh;
        }
        void setH(const shared_ptr<H> _H){H_ptr_ = _H;}
        list<shared_ptr<p>>& getplist() {return p_list_;}
        void addp(shared_ptr<p> _p);
};

class p : public enable_shared_from_this<p>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<S> S_ptr_;
    public:
        p(){cout << "construct p" << endl;}
        ~p(){cout << "destruct p" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getS()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<S> getS(){
            shared_ptr<S> S_sh = S_ptr_.lock();
            // if (S_sh) // check will be made by caller after all
            return S_sh;
        }
        void setS(const shared_ptr<S> _S){S_ptr_ = _S;}
};

class T : public enable_shared_from_this<T>
{
        weak_ptr<P> P_ptr_;
        list<shared_ptr<F>> F_list_;
    public:
        T(){cout << "construct T" << endl;}
        ~T(){cout << "destruct T" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (P_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<F>>& getFlist() {return F_list_;}
        void addF(shared_ptr<F> _F);
};

class F : public enable_shared_from_this<F>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<T> T_ptr_;
        list<shared_ptr<C>> C_list_;

        list<weak_ptr<c>> c_by_list; // list of constraints to this frame
    public:
        F(){cout << "construct F" << endl;}
        ~F(){cout << "destruct F" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getT()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<T> getT(){
            shared_ptr<T> T_sh = T_ptr_.lock();
            // if (T_sh) // check will be made by caller after all
            return T_sh;
        }
        void setT(const shared_ptr<T> _T){T_ptr_ = _T;}
        list<shared_ptr<C>>& getClist() {return C_list_;}
        void addC(shared_ptr<C> _C);
};

class C : public enable_shared_from_this<C>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<F> F_ptr_;
        list<shared_ptr<f>> f_list_;

        weak_ptr<S> S_ptr_; // sensor
    public:
        C(){cout << "construct C" << endl;}
        ~C(){cout << "destruct C" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getF()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<F> getF(){
            shared_ptr<F> F_sh = F_ptr_.lock();
            // if (F_sh) // check will be made by caller after all
            return F_sh;
        }
        void setF(const shared_ptr<F> _F){F_ptr_ = _F;}
        list<shared_ptr<f>>& getflist() {return f_list_;}
        void addf(shared_ptr<f> _f);
};

class f : public enable_shared_from_this<f>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<C> C_ptr_;
        list<shared_ptr<c>> c_list_; // change to list<weak_ptr<c>> ???

        list<weak_ptr<c>> c_by_list; // list of constraints to this feature

    public:
        f(){cout << "construct f" << endl;}
        ~f(){cout << "destruct f" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getC()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<C> getC(){
            shared_ptr<C> C_sh = C_ptr_.lock();
            // if (C_sh) // check will be made by caller after all
            return C_sh;
        }
        void setC(const shared_ptr<C> _C){C_ptr_ = _C;}
        list<shared_ptr<c>>& getclist() {return c_list_;} // Make a list of shareds from weaks? NO! then what?
        void addc(shared_ptr<c> _c); // use the same 'add' API, or transfer this to be mastered by the 'c' class?
};

class c : public enable_shared_from_this<c>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<f> f_ptr_; // change this to shared??

        // can we have just one pointer? Derive 3 classes from c?
        weak_ptr<F> F_other_ptr_; // change this to shared?
        weak_ptr<f> f_other_ptr_; // change this to shared?
        weak_ptr<L> L_other_ptr_; // change this to shared?
    public:
        c(){cout << "construct c" << endl;}
        ~c(){cout << "destruct c" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getf()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<f> getf(){
            shared_ptr<f> f_sh = f_ptr_.lock();
            // if (f_sh) // check will be made by caller after all
            return f_sh;
        }
        void setf(const shared_ptr<f> _f){f_ptr_ = _f;}
};

class M : public enable_shared_from_this<M>
{
        weak_ptr<P> P_ptr_;
        list<shared_ptr<L>> L_list_;
    public:
        M(){cout << "construct M" << endl;}
        ~M(){cout << "destruct M" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (P_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<L>>& getLlist() {return L_list_;}
        void addL(shared_ptr<L> _L);
};



class L : public enable_shared_from_this<L>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<M> M_ptr_;

        list<weak_ptr<c>> c_by_list; // list of constraints to this landmark

    public:
        L(){cout << "construct L" << endl;}
        ~L(){cout << "destruct L" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getM()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<M> getM(){
            shared_ptr<M> M_sh = M_ptr_.lock();
            // if (M_sh) // check will be made by caller after all
            return M_sh;
        }
        void setM(const shared_ptr<M> _M){M_ptr_ = _M;}
};

//////////////////////////////////////////

P::P(){
    cout << "construct P" << endl;
    H_ptr_ = make_shared<H>();
    T_ptr_ = make_shared<T>();
    M_ptr_ = make_shared<M>();
    cout << "P is constructed" << endl;
}

void P::setup()
{
    shared_ptr<P> pp = shared_from_this();
    H_ptr_->setP(shared_from_this());
    T_ptr_->setP(shared_from_this());
    M_ptr_->setP(shared_from_this());
    cout << "P is set up" << endl;
}

void H::addS(shared_ptr<S> _S)
{
    S_list_.push_back(_S);
    _S->setH(shared_from_this());
    _S->setP(getP());
}

void S::addp(shared_ptr<p> _p)
{
    p_list_.push_back(_p);
    _p->setS(shared_from_this());
    _p->setP(getP());
}

void T::addF(shared_ptr<F> _F)
{
    F_list_.push_back(_F);
    _F->setT(shared_from_this());
    _F->setP(getP());
}

void F::addC(shared_ptr<C> _C)
{
    C_list_.push_back(_C);
    _C->setF(shared_from_this());
    _C->setP(getP());
}

void C::addf(shared_ptr<f> _f)
{
    f_list_.push_back(_f);
    _f->setC(shared_from_this());
    _f->setP(getP());
}

void f::addc(shared_ptr<c> _c)
{
    c_list_.push_back(_c);
    _c->setf(shared_from_this());
    _c->setP(getP());
}

void M::addL(shared_ptr<L> _L)
{
    L_list_.push_back(_L);
    _L->setM(shared_from_this());
    _L->setP(getP());
}

}

using namespace wolf;
int main()
{
    shared_ptr<P> problem = make_shared<P>();
    problem->setup();

    // H
    problem->getH()->addS(make_shared<S>());
    problem->getH()->addS(make_shared<S>());
    for (auto S : problem->getH()->getSlist())
    {
        S->addp(make_shared<p>());
        S->addp(make_shared<p>());
    }

    // T
    problem->getT()->addF(make_shared<F>());
    problem->getT()->addF(make_shared<F>());
    for (auto F : problem->getT()->getFlist())
    {
        F->addC(make_shared<C>());
        F->addC(make_shared<C>());
        for (auto C : F->getClist())
        {
            C->addf(make_shared<f>());
            C->addf(make_shared<f>());
            for (auto f : C->getflist())
            {
                f->addc(make_shared<c>());
                f->addc(make_shared<c>());
            }
        }
    }

    // M
    problem->getM()->addL(make_shared<L>());
    problem->getM()->addL(make_shared<L>());

    cout << "Wolf tree created. Exiting main()." << endl;

    return 1;
}


