package locale;

//importa inet.mobility.IMobility;
import inet.mobility.models.MovingMobilityBase;

// Implementação da algoritmo de localização por nós vizinhos, LOCALE.
// NED que implementado o módulo de mobilidade do nó, necessitando de implementar a lógica do algoritmo
// na classe C++ correspondete ao  módulo
simple LOCALEMobility extends MovingMobilityBase
{
    parameters:
        double initialX @unit(m) = default(uniform(this.constraintAreaMinX, this.constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(this.constraintAreaMinY, this.constraintAreaMaxY));
        double initialZ @unit(m) = default(uniform(this.constraintAreaMinZ, this.constraintAreaMaxZ));
        double posicaoRealX @unit(m) = default(0);
        double posicaoRealY @unit(m) = default(0);
        double posicaoEstimadaX @unit(m) = default(0);
        double posicaoEstimadaY @unit(m) = default(0);
        double precisao = default(0);
        bool initFromDisplayString = default(true);
        double velocidade @unit(mps) = default(0mps); 
        @class(LOCALEMobility); // define a classe que implementará a lógica do módulo
}
