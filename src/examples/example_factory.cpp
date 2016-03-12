/**
 * \file example_factory.cpp
 *
 *  Created on: 09/03/2016
 *      \author: jsola
 */


#include "example_factory.h"

#include <list>

int main() {

    std::list<Shape*> shapes;

    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::POINT));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::LINE));

    for (auto shape : shapes){
        shape->draw();
    }
    (*(shapes.rbegin()))->draw();

    std::cout << "Asking for a CIRCLE which did not register to the factory..." << std::endl;
    shapes.push_back(ShapeFactory::instance()->createShape(Shape::CIRCLE));
    (*(shapes.rbegin()))->draw();

    return 0;
}
