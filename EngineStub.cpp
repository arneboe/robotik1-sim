#include "EngineStub.hpp"

#include "Robot.hpp"

extern "C" {
#include "../../engine.h"
}

extern "C" {
#include "../../odometry.h"
}

/**
 * Initializes internal data structures. 
 * */
void engine_init()
{
}


/**
 * This function advances the time of the engine by 
 * ms milliseconds.
 * After this it checks if the current sequence is 
 * done and if it needs to switch to the next step.
 * Finally it computes the servo angles for all 
 * servos and sets them.
 * @arg ms Time in ms that the engine will be moved forward
 * */
void engine_advance(int ms)
{
}


short unsigned int engine_getAngle(struct Step *curStep, short unsigned int time, enum Servos servo)
{
    return 0;
}

/**
 * This function sets a new step for the given index
 * 
 * @arg stepIndex The index at which the give step will be saved
 * @arg newData The step that will be saved at the given index.
 *      Note: The data will be copied, the pointer may be discarded
 *            after the function call. 
 * */
void engine_setNewStepData(unsigned int stepIndex, struct Step *newData)
{
}

/**
 * This function sets a new Sequence of steps that will
 * be executed. This function will override any previous
 * given step sequence, but will not stop the execution of
 * the current step. 
 * */
void engine_setNewStepSequence(struct StepSequence *sequence)
{
}

/**
 * Registeres a callback that will be called every time
 * a new step is executed.
 * */
void engine_registerStepChangeCallback(void (*callback)(unsigned int stepIndex))
{
}

/**
 * Returns if the current sequence is
 * finished.
 * @return 0 if not finished 1 if finished
 * */
unsigned char engine_isSequenceFinished()
{
}

/**
 * Returns if the current step is
 * finished.
 * @return 0 if not finished 1 if finished
 * */
unsigned char engine_isStepFinished()
{
}

/**
 * Initializes internal data structures
 * */
void odometry_init()
{
}

/**
 * This function sets the odometry for a given 
 * stepIndex. 
 * The given odometry should match the step that is
 * referenced by the stepIndex.
 * */
void odometry_setOdometryForStep(int stepIndex, struct OdometryOfStep *odmetry)
{
}

/**
 * Set the current position and orientation of 
 * the Robot. 
 * */
void odometry_setCurPose(struct Transform2D *pose)
{
}

/**
 * This function computes the next position
 * of the robot unter the assumption that 
 * a step, reference by stepindex, was 
 * executed.
 * */
void odometry_calculateNewPosition(unsigned int stepIndex)
{
}

/**
 * Returns the odometry position and rotation
 * of the robot.
 * */
struct Transform2D *odometry_getPose()
{
}