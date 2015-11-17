// implemtação da lógica de localização dos nós, através da impletação do algoritmo LOCALE
// algoritmo para estimativa de localização com colaboração de baixa densidade para redes móveis esparsas
// o próprio nó estima sua localização com base nos vizinhos, e colaborativamente refine essa localização
// para ser usado pelos vizinhos
#include "LOCALEMobility.h"
#include "FWMath.h"
#include <iostream>
#include <string>

using namespace std;

Define_Module(LOCALEMobility);

LOCALEMobility::LOCALEMobility() {
    speed = 0;
    angle = 0;
    acceleration = 0;
}

void LOCALEMobility::initialize(int stage) {
    MovingMobilityBase::initialize(stage);
    EV << "initializing LinearMobility stage " << stage << endl;
    if (stage == 0) {
        speed = par("speed");
        angle = fmod((double) par("angle"), 360);
        acceleration = par("acceleration");
        stationary = (speed == 0) && (acceleration == 0.0);
    }
}

void LOCALEMobility::move() {

    double rad = PI * angle / 180;

    Coord direction(cos(rad), sin(rad));
    lastSpeed = direction * speed;
    double elapsedTime = (simTime() - lastUpdate).dbl();
    lastPosition += lastSpeed * elapsedTime;
    cModule* module = this->getParentModule();

    for (cModule::GateIterator i(module); !i.end(); i++) {
        cGate *gate = i();
        cGate* g = gate->getPathEndGate();
        cModule* c = g->getOwnerModule();
        cModule* t = c->getParentModule();
        cout << t->getFullName() << "\n";
        cout << c->getFullName() << "\n";
        cout << gate->getFullName() << "\n";

    }

    // cout << module->gateSize("radioIn")<< "\n";
    //cout << module->gateSize("in30-1") << "\n";

    // do something if we reach the wall
    Coord dummy;
    handleIfOutside(REFLECT, dummy, dummy, angle);

    // accelerate
//       speed += acceleration * elapsedTime;
//       if (speed <= 0)
//       {
//           speed = 0;
//           stationary = true;
//       }
    //EV << " t= " << SIMTIME_STR(simTime()) << " xpos= " << lastPosition.x << " ypos=" << lastPosition.y << " speed=" << speed << endl;
}
