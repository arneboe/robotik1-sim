#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>

class StepInfo
{
public:
    Eigen::Vector3d posChange;
    Eigen::Vector3d posChangeVariance;
    double dirChange;
    double dirChangeVariance;
};

class Robot
{
private:
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
public:
    Robot();

    void setOrientation(const Eigen::Quaterniond &ori);
    const Eigen::Quaterniond &getOrientation() const;
    
    void setPosition(const Eigen::Vector3d &position);
    const Eigen::Vector3d &getPosition() const;
};

#endif // ROBOT_HPP
