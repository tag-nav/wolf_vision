#include <iostream>
#include "wolf.h"

using namespace std;
using namespace Eigen;

// We show how to map over maps
// The result maps to local storage, so changing the pointers of the first map does not modify the pointers of the second one.
int main()
{
    cout << "\nMap and remap - demo";
    cout << "\n--------------------\n" << endl;
    VectorXs X(15);
    X << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14;
    cout << "   X_: " << X.transpose() << endl;

    cout << "1. mapping x_ over the vector X_..." << endl;
//    Map<VectorXs> x(X.data() + 3, 6);
    Map<VectorXs> x(&X(3), 6);
    cout << "   x_: " << x.transpose() << endl;

    cout << "2. mapping p_ and v_ over the map x_:" << endl;
    Map<Vector3s> p(&x(0), 3), v(&x(0) + 3, 3);
    cout << "   (p,v)= (" << p.transpose() << " , " << v.transpose() << ")" << endl;

    cout << "3. altering the map v_ (out of x_, but inside X_):" << endl;
    new (&v) Map<Vector3s>(&x(0) + 6, 3);
    cout << "   (p,v)= (" << p.transpose() << " , " << v.transpose() << ")" << endl;

    cout << "4. altering the map x_ over another region of X_:" << endl;
    new (&x) Map<VectorXs>(&X(6), 6);
    cout << "   x_: " << x.transpose() << endl;
    cout << "   p_: " << p.transpose() << endl;

    cout << "5. ATTENTION!!! but the mapped maps p_ and v_ still point to the original data:" << endl;
    cout << "   (p,v)= (" << p.transpose() << " , " << v.transpose() << ")" << endl;

    cout << "6. altering the map v_ out of X_ (!!), gives unexpected data:" << endl;
    new (&v) Map<Vector3s>(&x(0) + 20, 3);
    cout << "   (p,v)= (" << p.transpose() << " , " << v.transpose() << ")" << endl;
    
    cout << "7. resizing the map x_ by 2:" << endl;
    //x.resize(x.size()+2);//Not allowed. Runtime error !
    new (&x) Map<VectorXs>(&x(0), x.size()+2);
    cout << "   x_: " << x.transpose() << endl;

    
    int num = -4294967296/2;
    cout << num << "; " << num-1 << "; " << num-(num-1) << endl;

    

    return 0;   
}

