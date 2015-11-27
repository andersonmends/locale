//// implemtação da lógica de localização dos nós, através da impletação do algoritmo LOCALE
//// algoritmo para estimativa de localização com colaboração de baixa densidade para redes móveis esparsas
//// o próprio nó estima sua localização com base nos vizinhos, e colaborativamente refine essa localização
//// para ser usado pelos vizinhos
#include "LOCALEMobility.h"
#include "FWMath.h"
#include <iostream>
#include <string>
#include <cmath>
#include <stdlib.h>

using namespace std;

Define_Module(LOCALEMobility);

LOCALEMobility::LOCALEMobility() {
    velocidade = 0;
    posicaoRealX = 0;
    posicaoRealY = 0;
    posicaoEstimadaX = 0;
    posicaoEstimadaY = 0;
    iniciador = 0;
    precisao = 0;

}

void LOCALEMobility::initialize(int stage) {
    LineSegmentsMobilityBase::initialize(stage);
//    EV << "initializing LOCALEMobility stage " << stage << endl;
    if (stage == 0) {
        velocidade = par("velocidade");
        stationary = velocidade == 0;
        posicaoRealX = lastPosition.x;
    }
}

void LOCALEMobility::setTargetPosition() {

    cModule* module = this->getParentModule();
    double mediaPrecisao = 0;
    int contadorMedia = 0;

// Implentação para simular a função de um nó móvel com dispositivo para detectar o range (alcance de um nó)
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

    if (strcmp(module->getFullName(), "node[0]") == 0) {

        cout << module->getFullName() << "\n";
        if (module->hasGate("out9-1")) {
            cModule* neighbor =
                    module->gate("out9-1")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";
            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;
            contadorMedia++;

        }

        if (module->hasGate("out9-2")) {
            cModule* neighbor =
                    module->gate("out9-2")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";
            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;
            contadorMedia++;
        }

        targetPosition = getRandomPosition();
        targetPosition.z = 0;
        double distancia = lastPosition.distance(targetPosition);
        nextChange = simTime() + distancia / velocidade;

        localPhase(distancia);

        cout << "Posição real: x " << targetPosition.x << " y "
                << targetPosition.y << "\n";
        cout << "Posição estimada: x " << posicaoEstimadaX << " y "
                << posicaoEstimadaY << "\n";
              if (contadorMedia !=0) {
                  cout << "Média da precisão: " << mediaPrecisao / contadorMedia << "\n";
                }

    }

    if (strcmp(module->getFullName(), "node[1]") == 0) {
        cout << module->getFullName() << "\n";
        if (module->hasGate("out16-1")) {
            cModule* neighbor =
                    module->gate("out16-1")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";

            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;

        }

        if (module->hasGate("out16-2")) {
            cModule* neighbor =
                    module->gate("out16-2")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";

            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;

        }

        targetPosition = getRandomPosition();
        targetPosition.z = 0;
        double distancia = lastPosition.distance(targetPosition);
        nextChange = simTime() + distancia / velocidade;
        localPhase(distancia);
        cout << "Posição real: x " << targetPosition.x << " y "
                << targetPosition.y << "\n";
        cout << "Posição estimada: x " << posicaoEstimadaX << " y "
                << posicaoEstimadaY << "\n";

        if (contadorMedia !=0) {
                         cout << "Média da precisão: " << mediaPrecisao / contadorMedia << "\n";
                       }

    }
    if (strcmp(module->getFullName(), "node[2]") == 0) {
        cout << module->getFullName() << "\n";
        if (module->hasGate("out23-1")) {
            cModule* neighbor =
                    module->gate("out23-1")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";

            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;

        }
        if (module->hasGate("out23-2")) {
            cModule* neighbor =
                    module->gate("out23-2")->getNextGate()->getOwnerModule();
            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
            cout << "Módulo " << module->getFullPath() << "está com "
                    << submoduleMobility->getFullPath() << " como vizinho"
                    << "\n";
            double p = submoduleMobility->par("precisao");
            cout << "Precisão do vizinho: " << p << "\n";
            mediaPrecisao = p + mediaPrecisao;

        }

        targetPosition = getRandomPosition();
        targetPosition.z = 0;
        double distancia = lastPosition.distance(targetPosition);
        nextChange = simTime() + distancia / velocidade;

        localPhase(distancia);

        cout << "Posição real: x " << targetPosition.x << " y "
                << targetPosition.y << "\n";
        cout << "Posição estimada: x " << posicaoEstimadaX << " y "
                << posicaoEstimadaY << "\n";
        if (contadorMedia !=0) {
                         cout << "Média da precisão: " << mediaPrecisao / contadorMedia << "\n";
                       }
    }

    cout << "\n";
}

void LOCALEMobility::localPhase(double distancia) {
    if (lastPosition.x < targetPosition.x) {
        posicaoEstimadaX = (distancia + (velocidade * velocidade)) / 2
                + lastPosition.x;
        par("posicaoEstimadaX").setDoubleValue(posicaoEstimadaX);
    } else {
        posicaoEstimadaX = abs(
                (distancia + (velocidade * velocidade * 3)) / 2
                        - lastPosition.x);
        par("posicaoEstimadaX").setDoubleValue(posicaoEstimadaX);
    }

    if (lastPosition.y < targetPosition.y) {
        posicaoEstimadaY = ((targetPosition.y - lastPosition.y)
                + (velocidade * velocidade)) + distancia;
        par("posicaoEstimadaY").setDoubleValue(posicaoEstimadaY);
    } else {
        posicaoEstimadaY = abs(distancia - lastPosition.y);
        par("posicaoEstimadaY").setDoubleValue(posicaoEstimadaY);

    }
    calcularPrecisao();
}

void LOCALEMobility::calcularPrecisao() {

    precisao = 100
            - abs(
                    ((posicaoEstimadaX - targetPosition.x)
                            / (double) targetPosition.x) * 100);
    precisao = abs(precisao);
    cout <<"random "<<rand()%100<<"\n";
    par("precisao").setDoubleValue(precisao);

}

void LOCALEMobility::mergePhase() {

}

