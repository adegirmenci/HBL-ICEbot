#ifndef SWEEP_H
#define SWEEP_H

#include <QObject>
#include <QElapsedTimer>
#include "kinematics_4dof.h"
#include "../icebot_definitions.h"

class Sweep
{
public:
    explicit Sweep(unsigned int nSteps_ = 30,
                   double stepAngle_ = 2.0*piOverDeg180,
                   double convLimit_ = 1.0*piOverDeg180,
                   qint64 imgDuration_ = 3000);

    void activate();
    void abort() {reset();}
    void reset();

    int update(const double currPsy);
    double getStepAngle() { return stepAngle; }

    unsigned int getRemainingSteps() { return remSteps; }
    bool getIsConverged() { return isConverged; }
    bool getIsActive() { return isActive; }
    unsigned int getNumControlCycles() { return nControlCycles; }
    qint64 getOverallTimeElapsed() { return overallTimer.elapsed(); }

private:
    bool isActive; // are we currently sweeping?
    unsigned int nControlCycles; // how many control cycles have elapsed since the beginning of this sweep
    unsigned int nSteps;   // number of desired steps (angle increments)
    unsigned int remSteps; // remaining number of steps (angle increments)
    double stepAngle;      // angle increment between sweps
    double convergenceLimit; // convergence criterion (CURR_PSY angle from 0)
    bool isConverged; // did we converge?
    qint64 imagingDuration; // how many ms to image for at current angle once converged
    QElapsedTimer timer;  // keep track of ms spent at current angle after convergence
    QElapsedTimer overallTimer;
};

#endif // SWEEP_H
