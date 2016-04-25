/*
 * \example_factory.h
 *
 *  Created on: 11/03/2016
 *      \author: jsola
 */

#ifndef EXAMPLE_FACTORY_H_
#define EXAMPLE_FACTORY_H_

#include <map>

class Shape // SensorBase
{
    public:
        typedef enum
        {
            POINT, LINE, CIRCLE
        } Type;
    public:
        Shape(int _id) : id_(_id){}
        virtual ~Shape();
        virtual void draw() const = 0;
    protected:
        int id_;
};

class Point : public Shape // SensorCamera
{
    public:
        Point();
        virtual ~Point();
        virtual void draw() const;
        Shape* createPoint();
    private:
        static int id_count_;
};

class Line : public Shape // SensorIMU
{
    public:
        Line();
        virtual ~Line();
        virtual void draw() const;
    private:
        static int id_count_;
};

class ShapeFactory // SensorFactory
{
    public:
        typedef Shape* (*CreateShapeCallback)();
    private:
        typedef std::map<Shape::Type, CreateShapeCallback> CallbackMap;
    public:
        // Returns 'true' if registration was successful
        bool registerShape(Shape::Type _shape_type, CreateShapeCallback createFn);
        // Returns 'true' if the _shape_type was registered before
        bool unregisterShape(Shape::Type _shape_type);
        Shape* createShape(Shape::Type _shape_type);
    private:
        CallbackMap callbacks_;

        // Singleton
    public:
        static ShapeFactory* instance(); // Unique point of access;
    private:
        ShapeFactory()
        {
        }
        // Prevent clients from creating a new Singleton
        ShapeFactory(const ShapeFactory&)
        {
        }
        // Prevent clients from creating a copy of the Singleton
        static ShapeFactory* pInstance_;
};

// IMPLEMENTATION //

#include <iostream>
#include <stdexcept>

// Point ----------------------------------------------------------------------

int Point::id_count_ = 0;

inline Point::Point() :
        Shape(++id_count_)
{
}

inline Point::~Point()
{
}

inline Shape::~Shape()
{
}

inline void Point::draw() const
{
    std::cout << "point " << id_ << std::endl;
}

inline Shape* Point::createPoint()
{
    return new Point;
}

namespace
{
Shape* CreatePoint()
{
    return new Point;
}
//const bool registered = ShapeFactory::instance()->registerShape(Shape::POINT, CreatePoint);
const bool registered = ShapeFactory::instance()->registerShape(Shape::POINT, CreatePoint);
}

// Line -----------------------------------------------------------------------

int Line::id_count_ = 0;

inline Line::Line() :
        Shape(++id_count_)
{
}

inline Line::~Line()
{
}

inline void Line::draw() const
{
    std::cout << "line  " << id_ << std::endl;
}

namespace
{
Shape* CreateLine()
{
    return new Line;
}
bool registered2 = ShapeFactory::instance()->registerShape(Shape::LINE, CreateLine);
}

// ShapeFactory ---------------------------------------------------------------

ShapeFactory* ShapeFactory::pInstance_ = nullptr;

inline ShapeFactory* ShapeFactory::instance() // Unique point of access
{
    if (!pInstance_)
        pInstance_ = new ShapeFactory;

    return pInstance_;
}

inline bool ShapeFactory::registerShape(Shape::Type _shape_type, CreateShapeCallback createFn)
{
    return callbacks_.insert(CallbackMap::value_type(_shape_type, createFn)).second;
}

inline bool ShapeFactory::unregisterShape(Shape::Type _shape_type)
{
    return callbacks_.erase(_shape_type) == 1;
}

inline Shape* ShapeFactory::createShape(Shape::Type _shape_type)
{
    CallbackMap::const_iterator i = callbacks_.find(_shape_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Shape type");
    }
    // Invoke the creation function
    return (i->second)();
}

#endif /* EXAMPLE_FACTORY_H_ */
