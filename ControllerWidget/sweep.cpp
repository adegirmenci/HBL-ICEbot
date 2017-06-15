#include "sweep.h"

Sweep::Sweep(unsigned int nSteps_,
             double stepAngle_,
             double convLimit_,
             qint64 imgDuration_):
    isActive(false),
    nControlCycles(0),
    remSteps(nSteps_),
    nSteps(nSteps_),
    stepAngle(stepAngle_),
    convergenceLimit(convLimit_),
    isConverged(false),
    imagingDuration(imgDuration_)
{ }

void Sweep::activate()
{
    overallTimer.start();
    reset();
    isActive = true;
}

void Sweep::reset()
{
    isActive = false;
    nControlCycles = 0;
    remSteps = nSteps;
    isConverged = false;
}

// reports required psy increment in radians
int Sweep::update(const double currPsy)
{
    if(isActive) // sweeping
    {
        nControlCycles++; // increment counter
        if(isConverged) // converged to target
        {
            if(timer.hasExpired(imagingDuration)) // done with this step
            {
                remSteps--;
                if(remSteps < 1) // done with sweep
                {
                    reset(); // reset counters
                    return SWEEP_DONE;
                }
                else
                {
                    isConverged = false;
                    return SWEEP_NEXT;
                }
            }
            else // still taking images
                return SWEEP_CONVERGED_ACQUIRING; // stay in place
        }
        else // not converged
        {
            if(abs(currPsy) < convergenceLimit) // converged!
            {
                isConverged = true;
                timer.start(); // start counting down
                return SWEEP_CONVERGED;
            }
            else // twiddle thumbs
            {
                return SWEEP_WAIT_TO_CONVERGE;
            }
        }
    }
    else // not even active
        return SWEEP_INACTIVE;
}
