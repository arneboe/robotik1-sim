#include "Robot.hpp"

Robot::Robot() : position(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity())
{
    position.z() = 0.1;
    orientation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
}

const Eigen::Quaterniond& Robot::getOrientation() const
{
    return orientation;
}

const Eigen::Vector3d& Robot::getPosition() const
{
    return position;
}

void Robot::setOrientation(const Eigen::Quaterniond& ori)
{
    orientation = ori;
}

void Robot::setPosition(const Eigen::Vector3d& position)
{
    this->position = position;
}
