<h3>Projeto executado na disciplina de Algoritmos Distribuídos, do Programa de Mestrado em Informática 
da Universidade Federal de Alagoas (UFAL). Que trata da implementação do algoritmo de estimativa de localização de redes móveis esparsas LOCALE (Collaborative Localization Estimation for Sparse Mobile Sensor Networks) através do uso do OMNET++.


<h3>1.	Introdução </h3>

O presente relatório trata do processo de implementação do algoritmo de estimativa de localização de redes móveis esparsas, por meio de colaboração entre os nós vizinhos, onde é refinada a sua estimativa de localização de acordo com a estimativa dos seus vizinhos. O algoritmo é chamado de LOCALE (Collaborative Localization Estimation for Sparse Mobile Sensor Networks) é apresentado no artigo de Zhang e Martonosi, do departamento de Engenharia Elétrica da Universidade de Princeton, na Conferência Internacional em Processamento de Informação em Redes Sensores, no ano de 2008. O objetivo do projeto é a implementação da lógica do LOCALE em um simulador de rede, para poder analisar a forma que foi implementado, o seu comportamento em relação a quantidade de nós e a precisão da estimativa de posição em determinado tempo. Para essa finalidade será usado OMNET++ com framework MIXIM, próprio para redes móveis ad-hoc, que nesse caso é o mais adequado.
 
<h3>2.	LOCALE </h3>

Basicamente para LOCALE prever e mesclar as informações de localização do nó, ele precisa da estimativa de localização e a certeza da estimativa, para isso LOCALE é divido em três fases. A fase local responsável pelo momento que o nó está desconectado dos nós vizinhos, nessa fase é usado um rastreamento de seu movimento para estimar sua localização. LOCALE tenta diminuir o uso de Dead Reckoning (DR) com alto custo, deixa o mais barato e trivial DR, isso junto com alguns cálculos da medida de distribuição relativa com a estimativa de distribuição, gerando uma nova distribuição a cada passo.
A segunda fase é a de transformação, aqui o nó recupera a informação da estimativa de localização dos vizinhos para depois refinar a sua estimativa e precisão, a partir da mesclagem das estimativas dos vizinhos, e para que seja apropriada para mesclagem, a estimativa do vizinho é transformada em observação, através de cálculos como a distância relativa entre dois nós, para trazer essa observação para o frame local do nó. LOCALE também tem várias formas de obter informação de localização entre dois nós, não só apenas através da distância relativa entre eles, e para os experimentos apresentado no artigo é assumido o caso mais simples, em que o nó pode discernir para outro nó no raio de comunicação, com uma distribuição de distância entre os nós.
A fase de atualização é a última, nessa fase o nó com as informações observadas anteriormente passa por um processo de mesclagem das estimativas dos vizinhos. É usada duas distribuições, a confiança e a estimativa de localização. As duas distribuições são usadas para gerar a nova estimativa e precisão do nó. 

<h3>3.	Implementação </h3>

De início, temos algumas considerações a fazer no desenvolvimento do projeto. Primeiro, na implementação foi considerada apenas duas dimensões, assim como nos experimentos do artigo, e o framework MiXiM do OMNET++ nos oferece a classe Coord com três dimensões, nós definimos a dimensão “Z” para todas as situações como “0”, apesar de poder ser expandido para usar três dimensões. 
Em um dos experimentos do artigo, é colocado um beacon no centro da área de simulação, para poder definir a posição inicial de cada nó, para nosso caso consideraremos a posição inicial definida pelo OMNET++.
Houve dificuldades para implementar a localização dinâmica dos nós através do método cModule::GateIterator, visto que, o OMNET++ apresentou um erro quando usado a função na estrutura for, presente em sua documentação, que possibilita iterar sobres os gates entre os vizinho em um determinado range, onde através desses gates seria possível estimar o alcance de cada nó em relação ao outro de forma dinâmica. Nesse caso, foi necessário improvisar através de estruturas for e if aninhadas para fazer a iteração entres os nós de forma dinâmica.
Outro aspecto que influenciou no desenvolvimento do projeto, foram os vários erros no momento de compilação do simulador, tendo várias vezes que fechar e abrir novamente o simulador.
Foi criada o módulo LOCALEMobility.ned e definida como sendo filha de MovingMobilityBase.ned, a primeira foi necessário definir alguns campos inerentes à LOCALE e a segunda oferece a base para se criar diversos tipos de movimentos diferentes do quem vem no MiXiM. Nesse caso é necessário criar o comportamento do módulo através da classe em C++ LOCALEMobility.cc, pela qual é implementado toda a lógica do sistema LOCALE.
Local Phase: a sua implementação depende uma um DR que pode ser usado em conjunto com sensores, o artigo faz um algoritmo junto com sensor de acelerômetro, no nosso caso foi necessário fazer algumas adaptações para OMNET++, já que não tivemos como simular o acelerômetro. O método localPhase() implementa essa fase, com um DR para estimar a posição e precisão do nó sem vizinhos, 
Transform Phase: os nós observam os vizinhos (recuperam) as suas estimativas e confiança. O método setTargetPosition() dá início a fase de observação, onde será disparado ou método localPhase() ou updatePhase() se houver vizinhos, se houver, ele vai recuperar as estmativas dos vizinhos. 
Update Phase: nessa fase é mesclado a estimativa dos vizinhos com sua confiança para refinar a nova estimativa do nó. Na nossa implementação essa fase é representada pelo método updatePhase(), onde é calculada a nova estimativa do nó através da confiança da estimativa dos vizinhos. 

<h3>4.	Resultados </h3>

Para executar as simulações foram definidos alguns parâmetros semelhantes aos escolhidos no artigo do sistema. O tempo de simulação foi definido para 8m como no artigo, ou seja, 480s. A área, semelhantes ao primeiro experimento, foi definido um campo de 1KM x 1KM. Foram escolhidos dois experimentos base, execução com 10 nós e com 100 nós.
Para o primeiro experimento foram geradas 174 amostras de estimativa de localização dos nós, delas foi extraída a melhor estimativa para cada nó e também a estimativa no final da execução, como pode ser observada na figura 01 e 02.

![image](https://user-images.githubusercontent.com/10708492/139906462-7600af46-45aa-460a-8521-c0babebce87d.png)

![image](https://user-images.githubusercontent.com/10708492/139906561-f08f79ec-2b8f-46f1-89f1-55786e32e0eb.png)

Para o segundo experimento, com bem mais nós, a quantidade de amostras foram 1838 amostras, como na anterior foi extraída a melhor estimativa para cada nó, como pode ser observada na figura 02.

![image](https://user-images.githubusercontent.com/10708492/139906619-e51aed4e-5160-48fa-8fc5-527d769f8204.png)


Para 10 nós a melhor estimativa deixa a estimativa bem próximo da posição real, já a estimativa final alguns nós mais isolados dos vizinhos ficaram com a estimativa um pouco distante do real.
Para 100 nós, tanto o ponto com a melhor estimativa quanto a final de comportaram de forma parecida, deixando a estimativa e real bem próximas. 
Como pode-se observar, a implementação de LOCALE tem uma precisão muito boa para um sistema que praticamente cálculos na localização no lugar de sensores. Para redes mais densas com mais vizinhos, a estimativa de localização é mais precisa do que uma rede com menos vizinhos.

