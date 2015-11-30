//// implemtação da lógica de localização dos nós, através da impletação do algoritmo LOCALE
//// o algoritmo para estimativa de localização com colaboração de baixa densidade para redes móveis esparsas
//// o próprio nó estima sua localização em fase de dead reckoning, chamada de local phaser,
// e após uma fase de obersvação dos vizinhos, observation phase, colaborativamente refine essa localização
//// com a mesclagem da estimativa de posição dos vizinhos, na merge phase
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

// método para inicar o LOCALE
// Implentação para simular a função de um nó móvel com dispositivo para detectar o range (alcance de um nó)
//lógica para iterar sobres os nós que estão próximos e então simular
// a fase inicial do LOCALE onde ele vai decidir se vai chamar a
// local phase ou merge phase, de se tem vizinhos ou não
void LOCALEMobility::setTargetPosition() {

    cModule* module = this->getParentModule();
    double mediaPrecisao = 0;
    int contadorMedia = 0;
    cModule* pai = module->getParentModule();
    int numNodes = pai->par("numNodes");

//Método para iterar sobre os gates de um nó, porém dando problema quando tenta acessar algum parametro de um submodule
    // foi necessário improvisar de outras forma para dá certo
//    for (cModule::GateIterator i(module); !i.end(); i++) {
//        cout << module->getFullName() << "\n";
//        cGate *gate = i();
//
//        cModule* neighbor = gate->getPathEndGate()->getOwnerModule();
//        cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//
//        cout << gate->getFullName() << "n/";
//
//        cout << "Módulo " << module->getFullPath() << "está com "
//                << submoduleMobility->getFullPath() << " como vizinho" << "\n";
//        double p = submoduleMobility->par("precisao");
//        cout << "Precisão do vizinho: " << p << "\n";
//        mediaPrecisao = p + mediaPrecisao;
//        contadorMedia++;
//
//    }


    for (int var = 0; var < numNodes; ++var) {

        std::string b = "node[" + std::to_string(var) + "]";
        const char * c = b.c_str();

        if (strcmp(module->getFullName(), c) == 0) {

            for (int i = 6 + numNodes; i < numNodes * 8; i = i + 7) {

                for (int j = 0; j < numNodes - 1; ++j) {
                    std::string gate = "out" + std::to_string(i) + "]" + "-"
                            + std::to_string(j);
                    const char * nomeGate = gate.c_str();
                    if (strcmp(module->getFullName(), nomeGate) == 0) {
                        cModule* neighbor =
                                module->gate(nomeGate)->getNextGate()->getOwnerModule();
                        cModule* submoduleMobility = neighbor->getSubmodule(
                                "mobility");
                        double p = submoduleMobility->par("precisao");
                        mediaPrecisao = p + mediaPrecisao;
                        contadorMedia++;
                    }
                }
            }

        }

    }

    if (contadorMedia == 0) {
        localPhase();
    } else {
        mergePhase(mediaPrecisao, contadorMedia);
    }

    // imprime no console a posição real, estimada e erro para poder fazer análise da simulação
    cout << simTime()<<"\n";
    cout << module->getFullName() << " " << posicaoRealX << " " << posicaoRealY <<"\n";
    cout << module->getFullName() << " " << posicaoEstimadaX << " " << posicaoEstimadaY <<"\n";




//    if (strcmp(module->getFullName(), "node[0]") == 0) {
//
//        cout << module->getFullName() << "\n";
//        if (module->hasGate("out9-1")) {
//            cModule* neighbor =
//                    module->gate("out9-1")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//            contadorMedia++;
//
//        }
//
//        if (module->hasGate("out9-2")) {
//            cModule* neighbor =
//                    module->gate("out9-2")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//            contadorMedia++;
//        }
//
//    if (contadorMedia == 0) {
//        localPhase();
//    } else {
//        mergePhase(mediaPrecisao, contadorMedia);
//    }
//
//    cout << "Posição real: x " << targetPosition.x << " y " << targetPosition.y
//            << "\n";
//    cout << "Posição estimada: x " << posicaoEstimadaX << " y "
//            << posicaoEstimadaY << "\n";
//
//    }
//
//    if (strcmp(module->getFullName(), "node[1]") == 0) {
//        cout << module->getFullName() << "\n";
//        if (module->hasGate("out16-1")) {
//            cModule* neighbor =
//                    module->gate("out16-1")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//
//        }
//
//        if (module->hasGate("out16-2")) {
//            cModule* neighbor =
//                    module->gate("out16-2")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//
//        }
//
//        if (contadorMedia == 0) {
//            localPhase();
//        } else {
//            mergePhase(mediaPrecisao, contadorMedia);
//        }
//
//        cout << "Posição real: x " << targetPosition.x << " y "
//                << targetPosition.y << "\n";
//        cout << "Posição estimada: x " << posicaoEstimadaX << " y "
//                << posicaoEstimadaY << "\n";
//
//    }
//    if (strcmp(module->getFullName(), "node[2]") == 0) {
//        cout << module->getFullName() << "\n";
//        if (module->hasGate("out23-1")) {
//            cModule* neighbor =
//                    module->gate("out23-1")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//
//        }
//        if (module->hasGate("out23-2")) {
//            cModule* neighbor =
//                    module->gate("out23-2")->getNextGate()->getOwnerModule();
//            cModule* submoduleMobility = neighbor->getSubmodule("mobility");
//            cout << "Módulo " << module->getFullPath() << "está com "
//                    << submoduleMobility->getFullPath() << " como vizinho"
//                    << "\n";
//            double p = submoduleMobility->par("precisao");
//            cout << "Precisão do vizinho: " << p << "\n";
//            mediaPrecisao = p + mediaPrecisao;
//
//        }
//
//        if (contadorMedia == 0) {
//            localPhase();
//        } else {
//            mergePhase(mediaPrecisao, contadorMedia);
//        }
//
//        cout << "Posição real: x " << posicaoRealX << " y " << posicaoRealY
//                << "\n";
//        cout << "Posição estimada: x " << posicaoEstimadaX << " y "
//                << posicaoEstimadaY << "\n";
//
//    }
//
//    cout << "\n";
}

// método que define a local phase, onde é implementado o DR quando não há vizinhos
void LOCALEMobility::localPhase() {
    targetPosition = getRandomPosition();
    targetPosition.z = 0;
    double distancia = lastPosition.distance(targetPosition);
    nextChange = simTime() + distancia / velocidade;
    posicaoRealX = targetPosition.x;
    posicaoRealY = targetPosition.y;
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

// método para cálcular a precisão daquele ciclo do LOCALE
void LOCALEMobility::calcularPrecisao() {
    double precisaoX;
    double precisaoY;
    if (posicaoEstimadaX > targetPosition.x) {
        precisaoX = abs(
                ((targetPosition.x - posicaoEstimadaX)
                        / (double) posicaoEstimadaX) * 100);
    } else {
        precisaoX = abs(
                ((posicaoEstimadaX - targetPosition.x)
                        / (double) targetPosition.x) * 100);
    }

    if (posicaoEstimadaY > targetPosition.y) {
        precisaoY = abs(
                ((targetPosition.y - posicaoEstimadaY)
                        / (double) posicaoEstimadaY) * 100);
    } else {
        precisaoY = abs(
                ((posicaoEstimadaY - targetPosition.y)
                        / (double) targetPosition.y) * 100);
    }

    precisao = (precisaoX + precisaoY) / 2;

    par("precisao").setDoubleValue(precisao);

}

// método para meclar a estimativa da posição do vizinho com o do nó, e então
// determinar a nova estimativa do nó
void LOCALEMobility::mergePhase(double mediaPrecisao, int contadorMedia) {
    targetPosition = getRandomPosition();
    targetPosition.z = 0;
    double distancia = lastPosition.distance(targetPosition);
    nextChange = simTime() + distancia / velocidade;
    posicaoRealX = targetPosition.x;
    posicaoRealY = targetPosition.y;
    int randomError = rand() % 100;

    if (randomError < 51) {
        posicaoEstimadaX = targetPosition.x - (mediaPrecisao / contadorMedia);
        par("posicaoEstimadaX").setDoubleValue(posicaoEstimadaX);
        posicaoEstimadaY = targetPosition.y - (mediaPrecisao / contadorMedia);
        par("posicaoEstimadaY").setDoubleValue(posicaoEstimadaY);
    } else {
        posicaoEstimadaX = targetPosition.x + (mediaPrecisao / contadorMedia);
        par("posicaoEstimadaX").setDoubleValue(posicaoEstimadaX);
        posicaoEstimadaY = targetPosition.y + (mediaPrecisao / contadorMedia);
        par("posicaoEstimadaY").setDoubleValue(posicaoEstimadaY);
    }
    calcularPrecisao();
}

