//// implemta��o da l�gica de localiza��o dos n�s, atrav�s da impleta��o do algoritmo LOCALE
//// algoritmo para estimativa de localiza��o com colabora��o de baixa densidade para redes m�veis esparsas
//// o pr�prio n� estima sua localiza��o com base nos vizinhos, e colaborativamente refine essa localiza��o
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

    for (cModule::GateIterator i(module); !i.end(); i++) {
        cGate *gate = i();
//        cGate* g = gate->getNextGate();
//        cGate* e = gate->getPreviousGate();
        cModule *neighbour = gate->getNextGate()->getOwnerModule();
//           cout << gate->getFullName() << "\n";
        cout << this->getFullPath() << "\n";
//        int t = neighbour->getId();
//        cout << t << "\n";

    }

//    Coord dummy;
//    angle = angle + 10;;
//    cout << this->getFullPath() << "\n";
////  cout << x << "\n";
//    cout << lastPosition << "\n";
//    cout << "" << "\n";
//
    targetPosition = getRandomPosition();
    Coord positionDelta = targetPosition - lastPosition;
    double distance = positionDelta.length();
    nextChange = simTime() + distance / speed;
//
//
//    cout <<this->getFullPath() << " new target set. distance=" << distance << " xpos= "
//              << targetPosition.x << " ypos=" << targetPosition.y
//              << " nextChange=" << nextChange << "\n\n";
}

