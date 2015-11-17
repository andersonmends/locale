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
        bool initFromDisplayString = default(true);
        double speed @unit(mps) = default(0mps); // speed of the host
        double angle @unit(deg) = default(uniform(0deg, 360deg)); // angle of linear motion
        double acceleration = default(0); // acceleration of linear motion (m/s2)
        @class(LOCALEMobility); // define a classe que implementará a lógica do módulo
}
