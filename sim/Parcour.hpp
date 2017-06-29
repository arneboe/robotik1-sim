#ifndef PARCOUR_HPP
#define PARCOUR_HPP

#include <Eigen/Geometry>
#include <vector>




class Box
{
public:
    double height;
    double width;
    double depth;

    //position at the middle of the object
    Eigen::Vector3d position;
    Eigen::Vector3d color;
    
    Eigen::AlignedBox<double, 2> bb;
};

class Cylinder
{
public:
    Eigen::Vector3d color;
    
    double height;
    double radius;
    //position at the middle of the object
    Eigen::Vector3d position;
};

class Parcour
{
    std::vector<Box> boxes;
    std::vector<Cylinder> cylinders;
    Eigen::Vector3d startPos1;
    Eigen::Vector3d startPos2;
public:
    const Eigen::Vector3d &getStartPoint1() const;
    const Eigen::Vector3d &getStartPoint2() const;
    const std::vector<Box> &getBoxes() const;
    const std::vector<Cylinder> &getCylinders() const;
    
    void addRightGoal(double x, double y);
    void addLeftGoal(double x, double y);
    void addLeftStart(double x, double y);
    void addRightStart(double x, double y);
    void addObstacle(double x, double y);
    void addWall(double x, double y);
};



#endif // PARCOUR_HPP
