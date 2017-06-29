#include "Simulator.hpp"
#include <string>
#include "Parser.hpp"
#include <iostream>
#include <stdlib.h>
#include "Robot.hpp"
#include <Eigen/Geometry>
#include <unistd.h>
#include <boost/thread/mutex.hpp>
#include <osgViewer/Viewer>

#define main robotMain
#include "../../main.cpp"
#undef main

#include "../../engine.hpp"
#include "../../bv_processor.hpp"

boost::mutex robotPositionLock;

Simulator *simP;

void simulator_advanceStep(uint8_t index, uint16_t timeInStep)
{
    simP->advanceStep(index, timeInStep);
}

Simulator* Simulator::getInstance()
{
    return simP;
}

void *robot_thread_func(void *)
{
    robotMain();
    return NULL;
}

extern Engine *engineG;

int main(int argc, char **argv)
{
    Simulator::Options opt;
    
    opt.pathToParcour = std::string("parcour.txt");
    opt.pathToStepInfo = std::string("stepinfo.txt");
    opt.startPos = Simulator::Options::START_LEFT;
    
    Simulator sim(opt);
    
    simP = &sim;
    

    //start robot thread
    std::cout << "Starting robot Thread" << std::endl;
    pthread_t robotThread;
    if(pthread_create(&robotThread, NULL, robot_thread_func, NULL))
    {
        std::cout << "Error Failed to create robot thread" << std::endl;
        exit(1);
    }

    while(!engineG)
        usleep(1000);

    //this is hacky, we depend on the fact, that
    //engine_initialize will not remove our callback.
    engineG->registerStepAdvanceCallback(CallBack<uint8_t,uint16_t>::getCallback(simulator_advanceStep));
    
    sim.run();
    
    return 0;
}

Visualization& Simulator::getVizualization()
{
    return *viz;
}

void Simulator::run()
{
    viz->setRobotPose(robot.getPosition(), robot.getOrientation());
    viz->getViewer()->getCamera()->setUpdateCallback(this);
    viz->getViewer()->run();
}


Simulator::Simulator(Simulator::Options& opt)
{
    Parser parser;
    parcour = parser.getParcour(opt.pathToParcour);
    if(!parcour)
    {
	std::cout << "Error could not parse parcour file" << std::endl;
	exit(1);
    }
    
    std::cout << "Parcout in " << std::endl;
    
    stepInfo = parser.getStepInfo(opt.pathToStepInfo);
    if(stepInfo.empty())
    {
        std::cout << "Error could not parse stepinfo file" << std::endl;
        exit(1);
    }
    
    if(opt.startPos == Options::START_LEFT)
    {
	robot.setPosition(parcour->getStartPoint1() + Eigen::Vector3d(0,0,0.1));
        robot.setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ())));
    } else 
    {
	robot.setPosition(parcour->getStartPoint2() + Eigen::Vector3d(0,0,0.1));
        robot.setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ())));
    }
    
    viz = new Visualization(*parcour);
}

void Simulator::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    //update camera Positions
    robotPositionLock.lock();
    viz->setRobotPose(robot.getPosition(), robot.getOrientation());
    robotPositionLock.unlock();
//     std::cout<<"Camera event callback - pre traverse"<<node<<std::endl;
//     traverse(node,nv);
//     std::cout<<"Camera event callback - post traverse"<<node<<std::endl;
    osg::NodeCallback::operator()(node, nv);
}

extern Odometry *odomtryG;

void Simulator::advanceStep(int index, int timeInStep)
{
    if(stepInfo.size() < index)
    {
        throw std::runtime_error("Error, no stepInfo give for step");
    }
    
    
    //new step
    if(timeInStep == 0)
    {
        odoIdxInStep = 0;
    }
    
    const OdometryOfStep &si = odometryG->getOdometryForStep(index);
    
    if(si.atTime[odoIdxInStep] >= timeInStep)
    {
        const StepInfo &sti(stepInfo[index]);
        
        Eigen::Vector2f trVariance = si.posDiff[odoIdxInStep] * sti.trError;
        trVariance.cwiseMax(sti.minTrVariance);
        double rotVariance = std::max((sti.dirChange * sti.rotError) / si.length, sti.minRotVariance);
        
        move(si.posDiff[odoIdxInStep].x(),si.posDiff[odoIdxInStep].y(),sti.dirChange / si.length,
            trVariance.x(), trVariance.y(), rotVariance);
        
        odoIdxInStep++;
//         std::cout << "Did Sub Step " << index << std::endl;
    }
    
}

class Random {
public:
    Random():haveNextGaussian(false)
    {
    }
    double gaussianRandom();
private:
    bool haveNextGaussian;
    double nextGaussian;
};

double Random::gaussianRandom()
{
   if (haveNextGaussian) {
       haveNextGaussian = false;
       return nextGaussian;
   } else {
       double v1, v2, s;
       do {
           v1 = ((2.0*random() / RAND_MAX - 1.0));   // between -1.0 and 1.0
           v2 = ((2.0*random() / RAND_MAX - 1.0));   // between -1.0 and 1.0
           s = v1 * v1 + v2 * v2;
       } while (s >= 1 || s == 0);
       double multiplier = sqrt(-2 * log(s)/s);
       nextGaussian = v2 * multiplier / 8;
       haveNextGaussian = true;
       
       return v1 * multiplier / 8;
   }
}

void Simulator::move(double x, double y, double angle, double varX, double varY, double varAngle)
{
//     std::cout << "Move " << x << " " << y << " Angle " << angle / M_PI * 180 << " Var " << varAngle  / M_PI * 180 <<  std::endl;
    Random r;
    double x_mov = x + r.gaussianRandom() * varX;
    double y_mov = y + r.gaussianRandom() * varY;
    double angle_mov = angle + r.gaussianRandom() * varAngle;
//     std::cout << "Move with Noise " << x_mov << " " << y_mov << " Angle " << angle_mov  / M_PI * 180 << std::endl;
    
    Eigen::Vector3d mov(x_mov, y_mov, 0);
    Eigen::Quaterniond rot(Eigen::AngleAxisd(angle_mov, Eigen::Vector3d::UnitZ()));
    mov = robot.getOrientation() * mov;
    Eigen::Vector3d nextPos3d = robot.getPosition() + mov;
//     std::cout << "Move Displ " << mov.transpose() << std::endl;
    
    //perform collision detection
    Eigen::Vector2d curPos = Eigen::Vector2d(robot.getPosition().x(), robot.getPosition().y());
    Eigen::Vector2d nextPos = Eigen::Vector2d(nextPos3d.x(), nextPos3d.y());
    Eigen::Vector2d diffPos = nextPos - curPos;

//     std::cout << "Moving from " << curPos.transpose() << " to " << nextPos.transpose() << std::endl; 
    
    //1cm steps
    double stepWidth = 0.01;
    double length = diffPos.norm();
    
    bool collision = false;

    //generate steps
    std::vector<double> steps;
    for(double curLength = 0; curLength < length; curLength += stepWidth)
    {
	steps.push_back(curLength);
    }
    double lastStep = fmod(length, stepWidth);
    if(lastStep != 0.0)
	steps.push_back(length);
    
    Eigen::Vector2d lastNonCollidingPos = curPos;
    
    for(std::vector<double>::const_iterator curStep = steps.begin(); curStep != steps.end() && !collision; curStep++)
    {
	Eigen::Vector2d stepPos = curPos + diffPos * (*curStep/length);
// 	std::cout << "StepPos " << stepPos.transpose() << std::endl;
	for(std::vector< ::Box >::const_iterator it = parcour->getBoxes().begin(); it != parcour->getBoxes().end();it++)
	{
	    if(it->bb.contains(stepPos))
	    {
		collision = true;
// 		std::cout << "Collision at:" << stepPos.transpose() << " with box " << it->position.transpose() << std::endl;

		break;
	    }
	}
	
	for(std::vector< ::Cylinder >::const_iterator it = parcour->getCylinders().begin(); it != parcour->getCylinders().end();it++)
	{
	    if((Eigen::Vector2d(it->position.x(), it->position.y()) - stepPos).norm() <= it->radius)
	    {
// 		std::cout << "Collision at:" << stepPos.transpose() << std::endl;
		collision = true;
		break;
	    }
	}
	if(!collision)
	    lastNonCollidingPos = stepPos;
    }

//     std::cout << "Collided:" << collision << std::endl;

    
    robotPositionLock.lock();
    robot.setPosition(Eigen::Vector3d(lastNonCollidingPos.x(), lastNonCollidingPos.y(), nextPos3d.z()));
    robot.setOrientation(robot.getOrientation() * rot);
    robotPositionLock.unlock();
}

const Eigen::Quaterniond &Simulator::getIMUOrientation() const {
    //maybe add some noise/drifts
    return robot.getOrientation();
}

