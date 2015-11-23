//// implemtação da lógica de localização dos nós, através da impletação do algoritmo LOCALE
//// algoritmo para estimativa de localização com colaboração de baixa densidade para redes móveis esparsas
//// o próprio nó estima sua localização com base nos vizinhos, e colaborativamente refine essa localização
//// para ser usado pelos vizinhos
//#include "LOCALEMobility.h"
//#include "FWMath.h"
//#include <iostream>
//#include <string>
//
//using namespace std;
//
//Define_Module(LOCALEMobility);
//
//LOCALEMobility::LOCALEMobility() {
//    speed = 0;
//    angle = 0;
//    acceleration = 0;
//}
//
//void LOCALEMobility::initialize(int stage) {
//    MovingMobilityBase::initialize(stage);
//    EV << "initializing LinearMobility stage " << stage << endl;
//    if (stage == 0) {
//        speed = par("speed");
//        angle = fmod((double) par("angle"), 360);
//        acceleration = par("acceleration");
//        stationary = (speed == 0) && (acceleration == 0.0);
//    }
//}
//
//void LOCALEMobility::move() {
//
//    double rad = PI * angle / 180;
//
//    Coord direction(cos(rad), sin(rad));
//    lastSpeed = direction * speed;
//    double elapsedTime = (simTime() - lastUpdate).dbl();
//    lastPosition += lastSpeed * elapsedTime;
//    cModule* module = this->getParentModule();
//
////    for (cModule::GateIterator i(module); !i.end(); i++) {
////        cGate *gate = i();
////        cGate* g = gate->getPathEndGate();
////        cModule* c = g->getOwnerModule();
////        cModule* t = c->getParentModule();
////        cout << t->getFullName() << "\n";
////        cout << c->getFullName() << "\n";
////        cout << gate->getFullName() << "\n";
////
////    }
//
//    // cout << module->gateSize("radioIn")<< "\n";
//    //cout << module->gateSize("in30-1") << "\n";
//
//    // do something if we reach the wall
//    Coord dummy;
//    handleIfOutside(REFLECT, dummy, dummy, angle);
//
//    // accelerate
////       speed += acceleration * elapsedTime;
////       if (speed <= 0)
////       {
////           speed = 0;
////           stationary = true;
////       }
//    //EV << " t= " << SIMTIME_STR(simTime()) << " xpos= " << lastPosition.x << " ypos=" << lastPosition.y << " speed=" << speed << endl;
//}

/***************************************************************************
 * file:        ConstSpeedMobility.cc
 *
 * author:      Steffen Sroka
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/

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
    posicaoAnterior = Coord(0,0);
}

void LOCALEMobility::initialize(int stage) {
    LineSegmentsMobilityBase::initialize(stage);
    EV << "initializing LOCALEMobility stage " << stage << endl;
    if (stage == 0) {
        speed = par("speed");
        stationary = speed == 0;
    }
}

void LOCALEMobility::setTargetPosition() {

    cModule* module = this->getParentModule();
    if (module->hasGate("out9-1")) {
        cModule* neighbor =
                module->gate("out9-1")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";



    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    if (module->hasGate("out9-2")) {
        cModule* neighbor =
                module->gate("out9-2")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";
    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    if (module->hasGate("out16-1")) {
        cModule* neighbor =
                module->gate("out16-1")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";

    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    if (module->hasGate("out16-2")) {
        cModule* neighbor =
                module->gate("out16-2")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";

    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    if (module->hasGate("out23-1")) {
        cModule* neighbor =
                module->gate("out23-1")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";

    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    if (module->hasGate("out23-2")) {
        cModule* neighbor =
                module->gate("out23-2")->getNextGate()->getOwnerModule();
        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
        cout << "Módulo " << module->getFullPath() << "está com "
                << submoduleMobility->getFullPath() << " como vizinho" << "\n";

    } else {
        cout << "Módulo " << module->getFullName() << "está sem vizinhos"
                << "\n";
    }

    // Está dando problema quando tenta acessar algum parametro do submodule
//        for (cModule::GateIterator i(module); !i.end(); i++) {
//            cGate *gate = i();
//
//            cModule* neighbor = gate->getPathEndGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//
////            cout << "Módulo " << module->getFullPath() << "está com "
////                    << submoduleMobility->getFullPath() << " como vizinho" << "\n";
//
//        }

    if (strcmp(module->getFullName(), "node[1]") == 0) {
        targetPosition = getRandomPosition();
        posicaoAnterior = targetPosition;
        Coord positionDelta = targetPosition - lastPosition;
        double distance = positionDelta.length();
        nextChange = simTime() + distance / speed;
        cout << posicaoAnterior <<"\n";
    } else {
        cout << posicaoAnterior <<"\n";
        targetPosition = getRandomPosition();
        Coord positionDelta = targetPosition - lastPosition;
        double distance = positionDelta.length();
        nextChange = simTime() + distance / speed;
    }

    cout << "\n";


}

