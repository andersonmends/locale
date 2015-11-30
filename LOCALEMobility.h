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

    /** m�todo para calcular a estimativa do n� atrav�s de algoritmo de deadreckning*/
    virtual void localPhase();

    /** m�todo para calcular a precis�o da estimativa*/
    virtual void calcularPrecisao();

    virtual void mergePhase(double mediaPrecisao, int contadorMedia);

public:
    LOCALEMobility();
};

#endif

