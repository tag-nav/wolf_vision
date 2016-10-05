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
#include <vector>
#include <iostream>
using std::list;
using std::vector;
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


//////////////////////////////////////////////////////////////////////////////////
// DECLARE WOLF TREE

class P : public enable_shared_from_this<P>
{
    private:
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
    private:
        weak_ptr<P> P_ptr_;
        list<shared_ptr<S>> S_list_;

    public:
        H(){cout << "construct H" << endl;}
        ~H(){cout << "destruct H" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<S>>& getSlist() {return S_list_;}
        shared_ptr<S> add_S(shared_ptr<S> _S);
};

class S : public enable_shared_from_this<S>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<H> H_ptr_;
        list<shared_ptr<p>> p_list_;

        //        list<shared_ptr<C>> C_list_; // List of all captures

    public:
        S(){cout << "construct ·-S" << endl;}
        ~S(){cout << "destruct ·-S" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
             if (!P_sh)
                 P_sh = getH()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<H> getH(){
            shared_ptr<H> H_sh = H_ptr_.lock();
            return H_sh;
        }
        void setH(const shared_ptr<H> _H){H_ptr_ = _H;}
        list<shared_ptr<p>>& getplist() {return p_list_;}
        shared_ptr<p> add_p(shared_ptr<p> _p);
};

class p : public enable_shared_from_this<p>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<S> S_ptr_;

    public:
        p(){cout << "construct   ·-p" << endl;}
        ~p(){cout << "destruct   ·-p" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getS()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<S> getS(){
            shared_ptr<S> S_sh = S_ptr_.lock();
            return S_sh;
        }
        void setS(const shared_ptr<S> _S){S_ptr_ = _S;}
};

class T : public enable_shared_from_this<T>
{
    private:
        weak_ptr<P> P_ptr_;
        list<shared_ptr<F>> F_list_;

    public:
        T(){cout << "construct T" << endl;}
        ~T(){cout << "destruct T" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<F>>& getFlist() {return F_list_;}
        shared_ptr<F> add_F(shared_ptr<F> _F);
};

class F : public enable_shared_from_this<F>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<T> T_ptr_;
        list<shared_ptr<C>> C_list_;

        list<shared_ptr<c>> c_by_list; // list of constraints to this frame

        bool is_deleting;

    public:
        F() : is_deleting(false){cout << "construct ·-F" << endl;}
        ~F(){cout << "destruct ·-F" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getT()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<T> getT(){
            shared_ptr<T> T_sh = T_ptr_.lock();
            return T_sh;
        }
        void setT(const shared_ptr<T> _T){T_ptr_ = _T;}
        list<shared_ptr<C>>& getClist() {return C_list_;}
        list<shared_ptr<c>>& getCbyList(){return c_by_list;}
        shared_ptr<C> add_C(shared_ptr<C> _C);
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
        void remove();

};

class C : public enable_shared_from_this<C>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<F> F_ptr_;
        list<shared_ptr<f>> f_list_;

        weak_ptr<S> S_ptr_; // sensor

    public:
        C(){cout << "construct   ·-C" << endl;}
        ~C(){cout << "destruct   ·-C" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getF()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<F> getF(){
            shared_ptr<F> F_sh = F_ptr_.lock();
            return F_sh;
        }
        void setF(const shared_ptr<F> _F){F_ptr_ = _F;}
        list<shared_ptr<f>>& getflist() {return f_list_;}
        shared_ptr<f> add_f(shared_ptr<f> _f);
        void remove();

};

class f : public enable_shared_from_this<f>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<C> C_ptr_;
        list<shared_ptr<c>> c_list_;

        list<shared_ptr<c>> c_by_list; // list of constraints to this feature

        bool is_deleting;

    public:
        f():is_deleting(false){cout << "construct     ·-f" << endl;}
        ~f(){cout << "destruct     ·-f" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getC()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<C> getC(){
            shared_ptr<C> C_sh = C_ptr_.lock();
            return C_sh;
        }
        void setC(const shared_ptr<C> _C){C_ptr_ = _C;}
        list<shared_ptr<c>>& getclist() {return c_list_;}
        list<shared_ptr<c>>& getCbyList(){return c_by_list;}
        shared_ptr<c> add_c(shared_ptr<c> _c);
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
        void remove();
};

class c : public enable_shared_from_this<c>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<f> f_ptr_; // change this to shared??

        // can we have just one pointer? Derive 3 classes from c?
        weak_ptr<F> F_other_ptr_; // change this to shared?
        weak_ptr<f> f_other_ptr_; // change this to shared?
        weak_ptr<L> L_other_ptr_; // change this to shared?

    public:
        enum{c0, cF, cf, cL} type;
        c(){type = c0; cout << "construct       ·-c" << endl;}
        c(shared_ptr<F> _F_other);
        c(shared_ptr<f> _f_other);
        c(shared_ptr<L> _L_other);
        ~c(){cout << "destruct       ·-c" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getf()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<f> getf(){
            shared_ptr<f> f_sh = f_ptr_.lock();
            return f_sh;
        }
        void setf(const shared_ptr<f> _f){f_ptr_ = _f;}
        shared_ptr<F> getFother(){return F_other_ptr_.lock();}
        shared_ptr<f> getfother(){return f_other_ptr_.lock();}
        shared_ptr<L> getLother(){return L_other_ptr_.lock();}
        void remove();
};

class M : public enable_shared_from_this<M>
{
    private:
        weak_ptr<P> P_ptr_;
        list<shared_ptr<L>> L_list_;

    public:
        M(){cout << "construct M" << endl;}
        ~M(){cout << "destruct M" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            return P_sh;
        }
        void setP(const shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<L>>& getLlist() {return L_list_;}
        shared_ptr<L> add_L(shared_ptr<L> _L);
};



class L : public enable_shared_from_this<L>
{
    private:
        weak_ptr<P> P_ptr_;
        weak_ptr<M> M_ptr_;

        list<shared_ptr<c>> c_by_list; // list of constraints to this landmark

        bool is_deleting;

    public:
        L():is_deleting(false){cout << "construct ·-L" << endl;}
        ~L(){cout << "destruct ·-L" << endl;}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            if (!P_sh)
                P_sh = getM()->getP();
            return P_sh;
        }
        void setP(const shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<M> getM(){
            shared_ptr<M> M_sh = M_ptr_.lock();
            return M_sh;
        }
        void setM(const shared_ptr<M> _M){M_ptr_ = _M;}
        list<shared_ptr<c>>& getCbyList(){return c_by_list;}
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
        void remove();
};

//////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION of some methods

P::P(){
    cout << "construct P" << endl;
    H_ptr_ = make_shared<H>();
    T_ptr_ = make_shared<T>();
    M_ptr_ = make_shared<M>();
    cout << "P is constructed" << endl;
}

void P::setup()
{
    H_ptr_->setP(shared_from_this());
    T_ptr_->setP(shared_from_this());
    M_ptr_->setP(shared_from_this());
    cout << "P is set up" << endl;
}

shared_ptr<S> H::add_S(shared_ptr<S> _S)
{
    S_list_.push_back(_S);
    _S->setH(shared_from_this());
    _S->setP(getP());
    return _S;
}

shared_ptr<p> S::add_p(shared_ptr<p> _p)
{
    p_list_.push_back(_p);
    _p->setS(shared_from_this());
    _p->setP(getP());
    return _p;
}

shared_ptr<F> T::add_F(shared_ptr<F> _F)
{
    F_list_.push_back(_F);
    _F->setT(shared_from_this());
    _F->setP(getP());
    return _F;
}

shared_ptr<C> F::add_C(shared_ptr<C> _C)
{
    C_list_.push_back(_C);
    _C->setF(shared_from_this());
    _C->setP(getP());
    return _C;
}
void F::remove()
{
    if (!is_deleting){
        is_deleting = true;
        shared_ptr<F> Fp = shared_from_this();
        while (!c_by_list.empty())
            c_by_list.front()->remove();
        getT()->getFlist().remove(Fp);
    }
}

shared_ptr<f> C::add_f(shared_ptr<f> _f)
{
    f_list_.push_back(_f);
    _f->setC(shared_from_this());
    _f->setP(getP());
    return _f;
}
void C::remove()
{
    shared_ptr<C> Cp = shared_from_this();
    getF()->getClist().remove(Cp);
    if (getF()->getClist().empty())
        getF()->remove();
}

shared_ptr<c> f::add_c(shared_ptr<c> _c)
{
    c_list_.push_back(_c);
    _c->setf(shared_from_this());
    _c->setP(getP());
    return _c;
}
void f::remove()
{
    if (!is_deleting){
        is_deleting = true;
        shared_ptr<f> fp = shared_from_this();
        getC()->getflist().remove(fp);
        if (getC()->getflist().empty())
            getC()->remove();
        while (!c_by_list.empty())
            c_by_list.front()->remove();
    }
}

c::c(shared_ptr<F> _F_other)
{
    cout << "construct       ·-c -> F" << endl;
    type = cF;
    F_other_ptr_ = _F_other;
}

c::c(shared_ptr<f> _f_other)
{
    cout << "construct       ·-c -> f" << endl;
    type = cf;
    f_other_ptr_ = _f_other;
}

c::c(shared_ptr<L> _L_other)
{
    cout << "construct       ·-c -> L" << endl;
    type = cL;
    L_other_ptr_ = _L_other;
}
void c::remove()
{
    shared_ptr<c> cp = shared_from_this();

    // remove own: feature
    getf()->getclist().remove(cp);
    if (getf()->getclist().empty())
        getf()->remove();

    // remove other: {Frame, feature, Landmark}
    switch (type){
        case c::cF:
            getFother()->getCbyList().remove(cp);
            if (getFother()->getCbyList().empty())
                getFother()->remove();
            break;
        case c::cf:
            getfother()->getCbyList().remove(cp);
            if (getfother()->getCbyList().empty())
                getfother()->remove();
            break;
        case c::cL:
            getLother()->getCbyList().remove(cp);
            if (getLother()->getCbyList().empty())
                getLother()->remove();
            break;
        default:
            break;
    }
}


shared_ptr<L> M::add_L(shared_ptr<L> _L)
{
    L_list_.push_back(_L);
    _L->setM(shared_from_this());
    _L->setP(getP());
    return _L;
}
void L::remove()
{
    if (!is_deleting){
        is_deleting = true;
        shared_ptr<L> Lp = shared_from_this();
        while (!c_by_list.empty())
            c_by_list.front()->remove();
        getM()->getLlist().remove(Lp);
    }
}



}

//////////////////////////////////////////////////////////////////////////////////
// HELPERS

using namespace wolf;

void print_cF(const shared_ptr<P>& Pp)
{
    cout << "Frame constraints" << endl;
    for (auto Fp : Pp->getT()->getFlist())
    {
        cout << "F: " << Fp.get() << endl;
        for (auto Cp : Fp->getCbyList())
        {
            cout << " -> c: " << Cp.get() << " -> " << Cp->getFother().get() << endl;
        }
    }
}

void print_cL(const shared_ptr<P>& Pp)
{
    cout << "Landmark constraints" << endl;
    for (auto Lp : Pp->getM()->getLlist())
    {
        cout << "L: " << Lp.get() << endl;
        for (auto Cp : Lp->getCbyList())
        {
            cout << " -> c: " << Cp.get() << " -> " << Cp->getLother().get() << endl;
        }
    }
}

void print_c(const shared_ptr<P>& Pp)
{
    cout << "All constraints" << endl;
    for (auto Fp : Pp->getT()->getFlist())
    {
        for (auto Cp : Fp->getClist())
        {
            for (auto fp : Cp->getflist())
            {
                for (auto cp : fp->getclist())
                {
                    if (cp)
                        switch (cp->type)
                        {
                            case c::cF:
                                cout << "c: " << cp.get() << " -> F " << cp->getFother() << endl;
                                break;
                            case c::cf:
                                cout << "c: " << cp.get() << " -> f " << cp->getfother() << endl;
                                break;
                            case c::cL:
                                cout << "c: " << cp.get() << " -> L " << cp->getLother() << endl;
                                break;
                            default:
                                cout << "Bad constraint" << endl;
                                break;
                        }
                }
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
// MAIN

int main()
{
    int N = 3;

    shared_ptr<P> Pp = make_shared<P>();
    Pp->setup();

    // H
    for (int Si = 0; Si < 2; Si++)
    {
        shared_ptr<S> Sp = Pp->getH()->add_S(make_shared<S>());
        for (int pi = 0; pi < 2; pi++)
        {
            shared_ptr<p> pp = Sp->add_p(make_shared<p>());
        }
    }

    // M
    for (int Li = 0; Li < 2; Li++)
    {
        shared_ptr<L> Lp = Pp->getM()->add_L(make_shared<L>());
    }

    // T
    list<shared_ptr<L>>::iterator Lit = Pp->getM()->getLlist().begin();
    vector<weak_ptr<F>> Fvec(N);
    for (int Fi = 0; Fi < N; Fi++)
    {
        shared_ptr<F> Fp = Pp->getT()->add_F(make_shared<F>());
        Fvec.at(Fi) = Fp;
        for (int Ci = 0; Ci < 2; Ci++)
        {
            shared_ptr<C> Cp = Fp->add_C(make_shared<C>());
            for (int fi = 0; fi < 1; fi++)
            {
                shared_ptr<f> fp = Cp->add_f(make_shared<f>());
                if (Ci || !Fi) // landmark constraint
                {
                    shared_ptr<c> cp = fp->add_c(make_shared<c>(*Lit));
                    (*Lit)->add_c_by(cp);
                    Lit++;
                    if (Lit == Pp->getM()->getLlist().end())
                        Lit = Pp->getM()->getLlist().begin();
                }
                else // motion constraint
                {
                    shared_ptr<F> Fp = Fvec.at(Fi).lock();
                    if (Fp)
                    {
                        shared_ptr<c> cp = fp->add_c(make_shared<c>(Fvec.at(Fi).lock()));
                        Fp->add_c_by(cp);
                    }
                    else
                        cout << "Could not constrain Frame" << endl;
                }
            }
        }
    }

    cout << "Wolf tree created ----------------------------" << endl;

    cout << "\nShowing constraints --------------------------" << endl;
    cout<<endl;
    print_cF(Pp);
    cout<<endl;
    print_cL(Pp);
    cout<<endl;
    print_c(Pp);

    cout << "\nRemoving constraints -------------------------" << endl;
    Pp->getT()->getFlist().front()->getClist().front()->getflist().front()->getclist().front()->remove();
    Pp->getT()->getFlist().front()->getClist().front()->getflist().front()->getclist().front()->remove();
    Pp->getT()->getFlist().back()->getClist().front()->getflist().front()->getclist().front()->remove();
    Pp->getT()->getFlist().front()->getClist().back()->getflist().front()->getclist().front()->remove();


    cout << "\nRemoving landmarks ---------------------------" << endl;

    cout << "\nRemoving frames ------------------------------" << endl;

    cout << "\nExiting main() -------------------------------" << endl;

    return 1;
}


