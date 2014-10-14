#include <iostream>

#include "wolf.h"
#include "quaternion_tools.h"
#include "state_pose.h"

using namespace std;
using namespace Eigen;

int main()
{
    cout << "\nQuaternions - demo";
    cout << "\n------------------\n" << endl;

    Quaternions q1(1,2,3,4), q2(4,3,2,1);
    q1.normalize();
    q2.normalize();
    Quaternions q3 = q1*q2;
    Quaternions q4(AngleAxiss(.1,Vector3s(1,0,0)));
    Vector3s w(1,2,3);
    WolfScalar dt(.5);
    Vector3s wdt(w*dt);
    Quaternions q5(AngleAxiss(wdt.norm(),wdt.normalized()));
    Quaternions q6 = q1 * Quaternions(AngleAxiss(wdt.norm(), wdt.normalized()));
    Quaternions q7(Wolf::quaternionFromVector(wdt));
    Quaternions q8 = q1 * Wolf::quaternionFromVector(wdt);
    Quaternions q9(Wolf::quaternionFromVector(Vector3s(0,0,1e-8)));
    cout << "q1: " << q1.coeffs().transpose() << endl;
    cout << "q2: " << q2.coeffs().transpose() << endl;
    cout << "q3: " << q3.coeffs().transpose() << endl;
    cout << "q4: " << q4.coeffs().transpose() << endl;
    cout << "q5: " << q5.coeffs().transpose() << endl;
    cout << "q6: " << q6.coeffs().transpose() << endl;
    cout << "q7: " << q7.coeffs().transpose() << endl;
    cout << "q8: " << q8.coeffs().transpose() << endl;
    cout << "q9: " << q9.coeffs().transpose() << endl;

    cout << "\nMapped quaternions product" << endl;
    VectorXs storage(14);
    storage = VectorXs::Random(14);
    Map<Quaternions> qm1(&storage(0)), qm2(&storage(4));
    qm1.normalize();
    qm2.normalize();
    cout << "qm1: " << qm1.coeffs().transpose()  << "\nqm2: " << qm2.coeffs().transpose() << endl;
    Quaternions qp(qm1 * qm2);
    Map<Quaternions> qmp(&storage(8));
    qmp = qm1*qm2;
    cout << "prod direct: " << (qm1 * qm2).coeffs().transpose() << endl;
    cout << "prod local : " << qp.coeffs().transpose() << endl;
    cout << "prod map   : " << qmp.coeffs().transpose() << endl;
    cout << "storage: " << storage.transpose() << endl;

    cout << "\nNow mapped and inside a class with getter, a la StatePose" << endl;
    class Q{
        private:
            VectorXs ql_;
            Map<Quaternions> qm_;
        public:
            Q(Quaternions & _q)      : ql_(4), qm_(&ql_(0))              { ql_ = _q.coeffs(); } // maps to local
            Q(Map<Quaternions> & _q) : ql_(), qm_((_q.coeffs().data())) { } // maps to remote
            Map<Quaternions>       & qm()       { return qm_; }
            const Map<Quaternions> & qm() const { return qm_; }
            Quaternions               q()       { return Quaternions(qm_); }
            const Quaternions         q() const { return Quaternions(qm_); }
    };

    // Now use the class and pointer
    Q qc1(qm1), qc2(qm2);
    Quaternions qcp = qc1.qm() * qc2.qm();
    qmp = qc1.qm() * qc2.qm();
    cout << "prod over local: " << qcp.coeffs().transpose() << endl;
    cout << "prod over map  : " << qmp.coeffs().transpose() << endl;
    cout << "storage: " << storage.transpose() << endl;

    // pointers
    cout << "\nNow with pointers to the classes, even more a la Wolf" << endl;
    typedef Q* Q_p;
    typedef Q_p* Q_pp;
    Q_p qc1_p = &qc1;
    Q_p qc2_p = &qc2;
    Q_pp qc1_pp = &qc1_p;
    Q_pp qc2_pp = &qc2_p;
    qmp = qc1_p->qm() * qc2_p->qm();
    cout << "prod of pointed Q's over map : " << qmp.coeffs().transpose() << endl;
    cout << "direct prod over cout        : " << (qc1_p->qm() * qc2_p->qm()).coeffs().transpose() << endl;
    cout << "double pointers              : " << ((*qc1_pp)->qm() * (*qc2_pp)->qm()).coeffs().transpose() << endl;


    // With StatePose
    cout << "\nNow with StatePose, even more a la Wolf" << endl;
    VectorXs storage2;
    storage2 = VectorXs::Random(14);
    StatePose sp1(storage2, 0, 7), sp2(storage2, 7, 7);
    sp1.q().normalize();
    sp2.q().normalize();
    qmp = sp1.q() * sp2.q();
    cout << "el producte de q maps a q map val: " << qmp.coeffs().transpose() << endl;
    cout << "el producte de q maps directe val: " << (sp1.q() * sp2.q()).coeffs().transpose() << endl;

    return 0;
}

