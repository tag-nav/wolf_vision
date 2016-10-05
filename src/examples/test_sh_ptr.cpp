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
            return T_sh;
        }
        void setT(const shared_ptr<T> _T){T_ptr_ = _T;}
        list<shared_ptr<C>>& getClist() {return C_list_;}
        shared_ptr<C> add_C(shared_ptr<C> _C);
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
};

class C : public enable_shared_from_this<C>
{
    private:
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
            return F_sh;
        }
        void setF(const shared_ptr<F> _F){F_ptr_ = _F;}
        list<shared_ptr<f>>& getflist() {return f_list_;}
        shared_ptr<f> add_f(shared_ptr<f> _f);
};

class f : public enable_shared_from_this<f>
{
    private:
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
            return C_sh;
        }
        void setC(const shared_ptr<C> _C){C_ptr_ = _C;}
        list<shared_ptr<c>>& getclist() {return c_list_;} // Make a list of shareds from weaks? NO! then what?
        shared_ptr<c> add_c(shared_ptr<c> _c); // use the same 'add' API, or transfer this to be mastered by the 'c' class?
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
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
        c(){cout << "construct c" << endl;}
        c(shared_ptr<F> _F_other);
        c(shared_ptr<f> _f_other);
        c(shared_ptr<L> _L_other);
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
            return f_sh;
        }
        void setf(const shared_ptr<f> _f){f_ptr_ = _f;}
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
        void removeL(shared_ptr<L> _L)
        {
            L_list_.remove(_L);
        }
};



class L : public enable_shared_from_this<L>
{
    private:
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
            return M_sh;
        }
        void setM(const shared_ptr<M> _M){M_ptr_ = _M;}
        void add_c_by(shared_ptr<c> _cby)
        {
            c_by_list.push_back(_cby);
        }
        void remove_c_by(shared_ptr<c> _cby)
        {
            c_by_list.remove_if( // remove _cby
                    [_cby](std::weak_ptr<c> p){
                        std::shared_ptr<c> sp = p.lock();
                        if(_cby && sp)
                            return _cby == sp;
                        return false;});
            if (c_by_list.empty())
                getM()->removeL(shared_from_this());
        }
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

shared_ptr<f> C::add_f(shared_ptr<f> _f)
{
    f_list_.push_back(_f);
    _f->setC(shared_from_this());
    _f->setP(getP());
    return _f;
}

shared_ptr<c> f::add_c(shared_ptr<c> _c)
{
    c_list_.push_back(_c);
    _c->setf(shared_from_this());
    _c->setP(getP());
    return _c;
}

c::c(shared_ptr<F> _F_other)
{
    F_other_ptr_ = _F_other;
}

c::c(shared_ptr<f> _f_other)
{
    f_other_ptr_ = _f_other;
}

c::c(shared_ptr<L> _L_other)
{
    L_other_ptr_ = _L_other;
}

shared_ptr<L> M::add_L(shared_ptr<L> _L)
{
    L_list_.push_back(_L);
    _L->setM(shared_from_this());
    _L->setP(getP());
    return _L;
}


}

//////////////////////////////////////////////////////////////////////////////////
// MAIN

using namespace wolf;
int main()
{
    shared_ptr<P> Pp = make_shared<P>();
    Pp->setup();

    // H
    for (int i = 0; i < 2; i++)
    {
        shared_ptr<S> Sp = Pp->getH()->add_S(make_shared<S>());
        for (int i = 0; i < 2; i++)
        {
            shared_ptr<p> pp = Sp->add_p(make_shared<p>());
        }
    }

    // M
    for (int i = 0; i < 2; i++)
    {
        shared_ptr<L> Lp = Pp->getM()->add_L(make_shared<L>());
    }

    // T
    for (int i = 0; i < 2; i++)
    {
        shared_ptr<F> Fp = Pp->getT()->add_F(make_shared<F>());
        for (int i = 0; i < 2; i++)
        {
            shared_ptr<C> Cp = Fp->add_C(make_shared<C>());
            for (int i = 0; i < 2; i++)
            {
                shared_ptr<f> fp = Cp->add_f(make_shared<f>());
                list<shared_ptr<L>>::iterator Lit = Pp->getM()->getLlist().begin();
                for (int i = 0; i < 2; i++)
                {
                    if(*Lit)
                    {
                        shared_ptr<c> cp = fp->add_c(make_shared<c>(*Lit));
                        (*Lit)->add_c_by(cp);
                    }
                    else
                        cout << "Could not constrain lmk" << endl;

                    Lit++;
                }
            }
        }
    }

    cout << "Wolf tree created." << endl;

    cout << "Removing constraints" << endl;

    cout << "Removing landmarks" << endl;

    cout << "Removing frames" << endl;

    cout << "Exiting main()." << endl;

    return 1;
}


