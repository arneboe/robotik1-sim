#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "Parcour.hpp"
#include <string>
#include "Robot.hpp"
#include "Visualization.hpp"
#include <osg/Node>

class Simulator : public osg::NodeCallback
{
public:
    class Options
    {
    public:
        std::string pathToParcour;
        std::string pathToStepInfo;
	enum StartPos {
	    START_LEFT,
	    START_RIGHT,
	};
	
	enum StartPos startPos;
    };
private:

    Parcour *parcour;
    Robot robot;
    Visualization *viz;
    int odoIdxInStep;

    std::vector< StepInfo > stepInfo;
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
    
public:
    Simulator(Options &opt);

    static Simulator *getInstance();
    
    void advanceStep(int index, int timeInStep);
    
    void move(double x, double y, double angle, double varX, double varY, double varAngle);

    const Eigen::Quaterniond &getIMUOrientation() const;

    /**
     * Blocks until the main windows is closed
     * */
    void run();
    
    Visualization &getVizualization();
};

#endif // SIMULATOR_HPP
