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
bool Sweep::update(const double currPsy)
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
                    return false;
                }
                else
                {
                    isConverged = false;
                    return true;
                }
            }
            else // still taking images
                return false; // stay in place
        }
        else // not converged
        {
            if(abs(currPsy) < convergenceLimit) // converged!
            {
                isConverged = true;
                timer.start(); // start counting down
                return false;
            }
            else // twiddle thumbs
            {
                return false;
            }
        }
    }
    else // not even active
        return false;
}
