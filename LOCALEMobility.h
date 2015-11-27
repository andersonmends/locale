//#ifndef LINEAR_MOBILITY_H
//#define LINEAR_MOBILITY_H
//
//#include "INETDefs.h"
//
//#include "MovingMobilityBase.h"
//
//class INET_API LOCALEMobility : public MovingMobilityBase
//{
//  protected:
//    double speed;          ///< speed of the host
//    double angle;          ///< angle of linear motion
//    double acceleration;   ///< acceleration of linear motion
//
//  protected:
//    /** @brief Initializes mobility model parameters.*/
//    virtual void initialize(int stage);
//
//    /** @brief Move the host*/
//    virtual void move();
//
//  public:
//    LOCALEMobility();
//};
//
//#endif

#ifndef CONST_SPEED_MOBILITY_H
#define CONST_SPEED_MOBILITY_H

#include "INETDefs.h"

#include "LineSegmentsMobilityBase.h"

/**
 * @brief Moves along a line with constant speed to a randomly chosen target.
 * When the target is reached it selects randomly a new one.
 *
 * @ingroup mobility
 * @author Steffen Sroka, Marc Loebbers, Daniel Willkomm
 */
class INET_API LOCALEMobility: public LineSegmentsMobilityBase {
protected:

    double velocidade;
    double posicaoRealX;
    double posicaoRealY;
    double posicaoEstimadaX;
    double posicaoEstimadaY;
    int iniciador;
    double precisao;

protected:
    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage);

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition();

    /** método para calcular a estimativa do nó através de algoritmo de deadreckning*/
    virtual void estimarPosition(double distancia);

    /** método para calcular a precisão da estimativa*/
       virtual void calcularPrecisao();

public:
    LOCALEMobility();
};

#endif

