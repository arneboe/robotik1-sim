#include "../../drivers/LSM9DS1/MadgwickAHRS.hpp"
#include "Simulator.hpp"
#include <math.h>

MadgwickFilter::MadgwickFilter(float beta, float sampleFreq)
  : beta(beta), sampleFreq(sampleFreq)
{
    //we ignore all of the parameters
}

extern Simulator *simP;

Eigen::Vector3f MadgwickFilter::getEuler() const
{
    const Eigen::Matrix3f m = simP->getIMUOrientation().toRotationMatrix().cast<float>();
    float x = Eigen::Vector2f(m.coeff(2,2) , m.coeff(2,1)).norm();
    Eigen::Vector3f res(0,::atan2f(-m.coeff(2,0), x),0);
    if (x > Eigen::NumTraits<float>::dummy_precision()){
        res[0] = ::atan2f(m.coeff(1,0), m.coeff(0,0));
        res[2] = ::atan2f(m.coeff(2,1), m.coeff(2,2));
    }else{
        res[0] = 0;
        res[2] = (m.coeff(2,0)>0?1:-1)* ::atan2f(-m.coeff(0,1), m.coeff(1,1));
    }

    return res;
}

float MadgwickFilter::getHeading() const
{
    float res = getEuler()[0];

    assert(!std::isnan(res));

    return res;
}

void MadgwickFilter::setInitialMagValues(float values[3])
{
    //not doing anything here
}

void MadgwickFilter::MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    //not here either
}

void MadgwickFilter::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    //or here
}
