#include "Parcour.hpp"


void Parcour::addLeftGoal(double x, double y)
{
    Cylinder c;
    c.color = Eigen::Vector3d(1,0,0);
    c.height = 0.4;
    c.position = Eigen::Vector3d(x,y,c.height/2);
    c.radius = 0.05;
    cylinders.push_back(c);
}

void Parcour::addRightGoal(double x, double y)
{
    Cylinder c;
    c.color = Eigen::Vector3d(0,0,1);
    c.height = 0.4;
    c.position = Eigen::Vector3d(x,y,c.height/2);
    c.radius = 0.05;
    cylinders.push_back(c);
}

void Parcour::addLeftStart(double x, double y)
{
    startPos1 = Eigen::Vector3d(x,y,0);
}

void Parcour::addRightStart(double x, double y)
{
    startPos2 = Eigen::Vector3d(x,y,0);
}

void Parcour::addObstacle(double x, double y)
{
    Box b;
    b.color = Eigen::Vector3d(0,1,0);
    b.depth = 0.1;
    b.height = 0.1;
    b.width = 0.1;
    b.position = Eigen::Vector3d(x,y,b.depth/2);
    b.bb.min() = Eigen::Vector2d(x - b.height / 2, y - b.height / 2);
    b.bb.max() = Eigen::Vector2d(x + b.width / 2, y + b.height / 2);
    boxes.push_back(b);
}

void Parcour::addWall(double x, double y)
{
    Box b;
    b.color = Eigen::Vector3d(1,1,1);
    b.depth = 0.8;
    b.height = 0.1;
    b.width = 0.1;
    b.position = Eigen::Vector3d(x,y,b.depth/2);
    b.bb.min() = Eigen::Vector2d(x - b.height / 2, y - b.height / 2);
    b.bb.max() = Eigen::Vector2d(x + b.width / 2, y + b.height / 2);
    boxes.push_back(b);
}

const std::vector< Box >& Parcour::getBoxes() const
{
    return boxes;
}

const std::vector< Cylinder >& Parcour::getCylinders() const
{
    return cylinders;
}

const Eigen::Vector3d& Parcour::getStartPoint1() const
{
    return startPos1;
}

const Eigen::Vector3d& Parcour::getStartPoint2() const
{
    return startPos2;
}

