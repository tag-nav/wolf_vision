#include <iostream>
#include <eigen3/Eigen/Dense>


#include "wolf.h"
#include "circular_buffer.h"

using namespace std;
using namespace Eigen;

int main()
{
    cout << "\nCircular buffer demo";
    cout << "\n--------------------\n" << endl;

    CircularBuffer<WolfScalar> cb(5);

    cout << "H: head, T: tail, S: size, F: full, E: empty; >-C-O-N-T-E-N-T-S->" << endl;

    WolfScalar n;
    cout << cb << endl;
    n = 0;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.push_head(++n);
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;
    cb.pop_tail();
    cout << cb << endl;

    cout << "----------------------------" << endl;
    return 0;
}

