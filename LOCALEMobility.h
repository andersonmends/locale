#ifndef LINEAR_MOBILITY_H
#define LINEAR_MOBILITY_H

#include "INETDefs.h"

#include "MovingMobilityBase.h"

class INET_API LOCALEMobility : public MovingMobilityBase
{
  protected:
    double speed;          ///< speed of the host
    double angle;          ///< angle of linear motion
    double acceleration;   ///< acceleration of linear motion

  protected:
    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int stage);

    /** @brief Move the host*/
    virtual void move();

  public:
    LOCALEMobility();
};

#endif
