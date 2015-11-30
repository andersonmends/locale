#ifndef CONST_SPEED_MOBILITY_H
#define CONST_SPEED_MOBILITY_H

#include "INETDefs.h"

#include "LineSegmentsMobilityBase.h"


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
    virtual void localPhase();

    /** método para calcular a precisão da estimativa*/
    virtual void calcularPrecisao();

    virtual void mergePhase(double mediaPrecisao, int contadorMedia);

public:
    LOCALEMobility();
};

#endif

