package locale;

//importa inet.mobility.IMobility;
import inet.mobility.models.MovingMobilityBase;

// Implementa��o da algoritmo de localiza��o atrav�s de mesclagem de posi��o dos vizinhos, LOCALE.
// NED que implementado o m�dulo de mobilidade do n�, necessitando implementar a l�gica do algoritmo
// na classe C++ correspondete ao  m�dulo
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
        @display("ls=,0");
        @class(LOCALEMobility); // define a classe que implementar� a l�gica do m�dulo
}
