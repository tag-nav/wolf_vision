
//std 
#include <iostream>
#include <vector>
#include <string>

//namespaces
using namespace std;

//classes
class Shape
{
    protected:
        unsigned int id_;
    
    public:
        Shape(const unsigned int _id) : 
            id_(_id) {};
        virtual ~ Shape() {};
        virtual double area() { return -1; }; 
        virtual string label() { return "SHAPE"; };
        void print() { cout << label() << " " << id_ << ", " << area() << endl; };
};

class Square : public Shape
{
    protected:
        double side_;
    
    public:
        Square(const unsigned int _id, const double _side) :
            Shape(_id), 
            side_(_side) {};
        ~Square() {};
        double area() { return side_*side_; };
        string label() { return "SQUARE"; };
};

class Circle : public Shape
{
    protected:
        double radius_;
    
    public:
        Circle(const unsigned int _id, const double _radius) :
            Shape(_id), 
            radius_(_radius) {};
        ~Circle() {};
        double area() { return radius_*radius_*3.141592; };
        string label() { return "CIRCLE"; };
};

int main()
{
    unsigned int nn=4;
    vector<Shape> shapeV;
    vector<Square> squareV;
    vector<Circle> circleV;
    vector<Shape*> shapePtrV;
    Square * squarePtr;
    Circle * circlePtr;
    
    cout << "Test 1: Pushing back Objects to dedicated vectors. homogeneous containers case." << endl;
    for (unsigned int ii=0; ii<nn; ii++)
    {
        shapeV.push_back( Shape(ii) );
        squareV.push_back( Square(ii+nn, (double)ii/(double)nn) );
        circleV.push_back( Circle(ii+nn*2, (double)ii/(double)nn) );
    }
    for (unsigned int ii=0; ii<nn; ii++) shapeV.at(ii).print();
    for (unsigned int ii=0; ii<nn; ii++) squareV.at(ii).print();
    for (unsigned int ii=0; ii<nn; ii++) circleV.at(ii).print();
    
    cout << "Test 2: Trying to push Squares and Circles to the Shape vector (vector with base class as a template parameter)" << endl;
    shapeV.clear();
    for (unsigned int ii=0; ii<nn; ii++)
    {
        if ( ii%2 == 0 ) shapeV.push_back( Square(ii+nn, (double)ii/(double)nn) );
        else shapeV.push_back( Circle(ii+nn, (double)ii/(double)nn) );
    }
    
    cout << "Test 2. Result A: Checking print() output when accessing to vector elements." << endl;
    for (unsigned int ii=0; ii<nn; ii++) shapeV.at(ii).print();
    
    cout << "Test 2. Result B: Checking which print() output by using pointers to inherited classes." << endl;
    for (unsigned int ii=0; ii<nn; ii++)
    {
        if ( ii%2 == 0 ) 
        {
            squarePtr = (Square *)( &shapeV.at(ii) );
            squarePtr->print();
        }
        else
        {
            circlePtr = (Circle *)( &shapeV.at(ii) );
            circlePtr->print();
        }        
    }

    cout << "Test 3. Using a vector of pointers." << endl;
    for (unsigned int ii=0; ii<nn; ii++)
    {
        if ( ii%2 == 0 ) shapePtrV.push_back( new Square(ii+nn, (double)ii/(double)nn) );
        else shapePtrV.push_back( new Circle(ii+nn, (double)ii/(double)nn) );
    }
    cout << "Test 3. Checking print() output." << endl;
    for (unsigned int ii=0; ii<nn; ii++)
    {
        shapePtrV.at(ii)->print();
    }
    
    cout << "Test 4. Valgrind test if clear() is correct" << endl;
    for (unsigned int ii=0; ii<shapePtrV.size(); ii++) delete shapePtrV.at(ii);
    shapePtrV.clear();
    
    return 0;   
}
