#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <iomanip>
#include <time.h>
#include <numeric>

using namespace std;

int Seed, Iteraciones_ILS, Iteraciones_SA, Iteraciones_CambioT, Movimientos_Perturbacion;
float Temperatura_Inicial, Factor_Enfriamiento;
char *Instancia;
unsigned t0, t1;

// *********************************************************************************************************** //
// ************************************************ FUNCIONES ************************************************ //
// *********************************************************************************************************** //

// Función obtenida del GitLab del profesor Nicolás Rojas para guardar los parámetros
void Capture_Params(int argc, char **argv){
    Instancia = argv[1];
    Seed = atoi(argv[2]); // atoi para int y atof para float
    Iteraciones_ILS = atoi(argv[3]);
    Iteraciones_SA = atoi(argv[4]);
    Iteraciones_CambioT = atoi(argv[5]);
    Movimientos_Perturbacion = atoi(argv[6]);
    Temperatura_Inicial = atof(argv[7]);
    Factor_Enfriamiento = atof(argv[8]);
}

// Función obtenida del GitLab del profesor Nicolás Rojas para seleccionar un entero aleatorio
int int_rand(int a, int b){
    int retorno = 0;

    if (a < b){
        retorno = (int) ((b - a) * drand48());
        retorno = retorno + a;
    }

    else{
        retorno = (int) ((a - b) * drand48());
        retorno = retorno + b;
    }

    return retorno;
}

// Función obtenida del GitLab del profesor Nicolás Rojas para seleccionar un flotante aleatorio
float float_rand(float a, float b){
    float retorno = 0;

    if (a < b){
        retorno = (float) ((b - a) * drand48());
        retorno = retorno + a;
    }

    else{
        retorno = (float) ((a - b) * drand48());
        retorno = retorno + b;
    }

    return retorno;
}

// Función para calcular la distancia entre dos puntos
float distancia(int x1, int y1, int x2, int y2){
    return sqrt( pow((x2-x1), 2) + pow((y2-y1), 2) );
}

// Función para encontrar un valor dentro de un vector
bool existeEnVector(vector<int> v, int busqueda){
    return find(v.begin(), v.end(), busqueda) != v.end();
}

// Función que calcula el consumo de energía entre dos nodos
float ConsumoEnergetico(int x1, int y1, int x2, int y2, float q){
    float Consumo;

    // Parámetros de la fórmula del consumo de energía
    float W = 2.5; // Peso de la estructura [kg]
    float m = 1.5; // Peso de la batería [kg]
    float g = 9.81; // Gravedad [N/kg]
    float ro = 1.204; // Densidad del fluido del aire [kg/m³]
    float sigma = 0.0064; // Área del disco de la hélice [m²]
    int h = 6; // Número de rotores del drone

    float dist = distancia(x1, y1, x2, y2);
    float t = dist;

    Consumo = pow((W+m+q), 1.5) * (sqrt(pow((g),3) / (2*ro*sigma*h))/1000) * (t/3600);
    return Consumo;
}

// Función para calcular la distancia total recorrida
float DistanciaTotal(vector<vector<int>> Rutas, vector<int> X_coor, vector<int> Y_coor){

    int i, j;
    float Distancia = 0;

    for(i=0;i<(int)Rutas.size();i++){
        for(j=0;j<(int)Rutas[i].size()-1;j++){
            Distancia += distancia(X_coor[Rutas[i][j]], Y_coor[Rutas[i][j]], X_coor[Rutas[i][j+1]], Y_coor[Rutas[i][j+1]]);
        }
    }
    
    return Distancia;
    
}

// Funcion que obtiene la cantidad de clientes visitados por cada drone
vector<int> ClientesDrones(vector<vector<int>> Rutas){
    int i, j, ContadorVisitados, NodoOrigen, NodoFinal;
    vector<int> VisitadosDrones;

    NodoOrigen = Rutas[0][0];
    NodoFinal = Rutas[0][Rutas[0].size()-1];

    for(i=0;i<(int)Rutas.size();i++){
        ContadorVisitados = 0;
        for(j=0;j<(int)Rutas[i].size();j++){
            if(Rutas[i][j] != NodoOrigen && Rutas[i][j] != NodoFinal){
                ContadorVisitados++;
            }
        }
        VisitadosDrones.push_back(ContadorVisitados);
    }
    return VisitadosDrones;
}

// Función que elige un drone en base a la cantidad de clientes visitados
int RuletaDrones(vector<int> VisitadosDrones){
    int i, j, ClientesTotales, NumeroAleatorio;
    vector<int> AparicionDrones;

    ClientesTotales = accumulate(VisitadosDrones.begin(), VisitadosDrones.end(), 0);

    for(i=0; i<(int)VisitadosDrones.size(); i++){
        for(j=0; j<VisitadosDrones[i]; j++){
            AparicionDrones.push_back(VisitadosDrones[i]);
        }
    }

    NumeroAleatorio = int_rand(0, ClientesTotales);

    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] == AparicionDrones[NumeroAleatorio])
            return i;
    }
    return i;
}

// Función que calcula la función de evaluación entre 2 nodos
float FuncionEvaluacionNodos(int Nodo1, int Nodo2, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){
    
    float ValorTotal = 0, tiempo, energia;

    if(tipo == 0){
        tiempo = distancia(X_coor[Nodo1], Y_coor[Nodo1], X_coor[Nodo2], Y_coor[Nodo2]);
        ValorTotal += tiempo;
    }
    else if(tipo == 1){
        energia = ConsumoEnergetico(X_coor[Nodo1], Y_coor[Nodo1], X_coor[Nodo2], Y_coor[Nodo2], q);
        ValorTotal += energia;
        // ValorTotal += delta * energia; (ANALIZAR SI MULTIPLICARLO POR DELTA)
    }
    else if(tipo == 2){
        tiempo = distancia(X_coor[Nodo1], Y_coor[Nodo1], X_coor[Nodo2], Y_coor[Nodo2]);
        energia = ConsumoEnergetico(X_coor[Nodo1], Y_coor[Nodo1], X_coor[Nodo2], Y_coor[Nodo2], q);
        ValorTotal += tiempo + (delta * energia);
    }

    return ValorTotal;

}

// Función que calcula la función de evaluación de una ruta
float FuncionEvaluacionRuta(vector<int> Rutas, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){
    int i, Largo = Rutas.size()-1;
    float ValorTotal = 0, tiempo, energia;
    for(i=0;i<Largo;i++){
        if(tipo == 0){
            tiempo = distancia(X_coor[Rutas[i]], Y_coor[Rutas[i]], X_coor[Rutas[i+1]], Y_coor[Rutas[i+1]]);
            ValorTotal += tiempo;
        }
        else if(tipo == 1){
            energia = ConsumoEnergetico(X_coor[Rutas[i]], Y_coor[Rutas[i]], X_coor[Rutas[i+1]], Y_coor[Rutas[i+1]], q);
            ValorTotal += delta * energia;
        }
        else if(tipo == 2){
            tiempo = distancia(X_coor[Rutas[i]], Y_coor[Rutas[i]], X_coor[Rutas[i+1]], Y_coor[Rutas[i+1]]);
            energia = ConsumoEnergetico(X_coor[Rutas[i]], Y_coor[Rutas[i]], X_coor[Rutas[i+1]], Y_coor[Rutas[i+1]], q);
            ValorTotal += tiempo + (delta * energia);
        }
    }
    return ValorTotal;
}

// Función que calcula la función de evaluación de todas las rutas
float FuncionEvaluacionTotal(vector<vector<int>> Rutas, int NumRutas, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){
    int i;
    float Resultado;
    for(i=0;i<NumRutas;i++){
        Resultado += FuncionEvaluacionRuta(Rutas[i], X_coor, Y_coor, delta, tipo, q);
    }
    return Resultado;
}

// Actualiza los tiempos
bool NuevosTiempos(vector<int> Ruta, vector<float> *TiemposNodos, vector<float> *EnergiaNodos, vector<int> NodeID, 
vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime, int Limite, float EnergiaMax, int delta, int tipo, float q){

    int i, Deposito = NodeID[0];
    float Distancia, ConsumoEnergia, EnergiaAcum;
    vector<float> Tiempos = *TiemposNodos;
    vector<float> Energia = *EnergiaNodos;

    int Largo = Ruta.size();

    for(i=Limite;i<Largo;i++){
        if(Ruta[i-1] == Deposito)
            EnergiaAcum = 0;
        else
            EnergiaAcum = Energia[i-1];
        Distancia = FuncionEvaluacionNodos(Ruta[i-1], Ruta[i], X_coor, Y_coor, delta, tipo, q);
        ConsumoEnergia = ConsumoEnergetico(X_coor[Ruta[i-1]], Y_coor[Ruta[i-1]], X_coor[Ruta[i]], Y_coor[Ruta[i]], q);
        EnergiaAcum += ConsumoEnergia;
        Tiempos[i] = Tiempos[i-1] + Distancia;
        Energia[i] = EnergiaAcum;
        // Si no me encuentro dentro de la ventana de tiempo retorno false
        if(Tiempos[i] < ReadyTime[Ruta[i]] || Tiempos[i] > DueTime[Ruta[i]] || Energia[i] > EnergiaMax){
            return false;
        }
    }
    
    // Si estoy dentro de la ventana de tiempo retorno true
    *TiemposNodos = Tiempos;
    *EnergiaNodos = Energia;
    return true;
}

// Función para leer la instancia y asignar los parámetros a sus respectivas variables
void leerInstancia(string NombreInstancia, int *CustNum, int *DroneNum, vector<int> *NodeID, 
vector<int> *X_coor, vector<int> *Y_coor, vector<float> *Demand, vector<int> *ReadyTime, vector<int> *DueTime){

    int countArray, TokenInt;
    float TokenFloat;
    ifstream instancia;
    string linea, token;
    size_t pos = 0;

    while(true){
        instancia.open("./Instancias/Type_1/" + NombreInstancia + ".txt");
        if(instancia)
            break;
    }
    
    // Se lee la línea del Número de Clientes
    string delimiter = "	";
    getline(instancia, linea);
    while ((pos = linea.find(delimiter)) != string::npos){
        token = linea.substr(0, pos);
        linea.erase(0, pos + delimiter.length());
    }
    *CustNum = stoi(linea);

    // Se lee la línea del Número de Drones
    getline(instancia, linea);
    while ((pos = linea.find(delimiter)) != string::npos){
        token = linea.substr(0, pos);
        linea.erase(0, pos + delimiter.length());
    }
    *DroneNum = stoi(linea);

    // Se leen las líneas de cada Cliente
    getline(instancia, linea);
    delimiter = "	";
    while (!instancia.eof()){
        countArray = 1;
        getline(instancia, linea);
        while ((pos = linea.find(delimiter)) != string::npos){

            token = linea.substr(0, pos);
            istringstream(token) >> TokenInt;
            istringstream(token) >> TokenFloat;

            if(countArray == 1){
                NodeID->push_back(TokenInt);
            }
                
            else if(countArray == 2){
                X_coor->push_back(TokenInt);
            }

            else if(countArray == 3){
                Y_coor->push_back(TokenInt);
            }

            else if(countArray == 4){
                Demand->push_back(TokenFloat);
            }

            else if(countArray == 5){
                ReadyTime->push_back(TokenInt);
            }
            
            linea.erase(0, pos + delimiter.length());
            countArray++;
        }
        istringstream(linea) >> TokenInt;
        DueTime->push_back(TokenInt);
    }
}

// Función que verifica que todos los clientes estén visitados (-1)
bool TodosVisitados(int DroneID[], int CustNum){
    int i;

    for(i=0;i<CustNum;i++){
        if(DroneID[i] != -1)
            return false;
    }
    return true;
}

// Función que verifica si el DroneActual está dentro de DroneID
bool VerificarDrone(int DroneID[], int CustNum, int DroneActual){
    int i;
    for(i=0;i<CustNum;i++){
        // Sigo con ese mismo drone
        if(DroneID[i] == DroneActual)
            return false;
    }
    return true;
}

// Función que selecciona un movimiento según sus probabilidades
int RuletaMovimientos(vector<float> ProbAcumuladas, float Random){
    int i, Movimiento;

    for(i=0;i<(int)ProbAcumuladas.size();i++){
        if(ProbAcumuladas[i] > Random){
            Movimiento = i;
            break;
        }
    }

    return Movimiento;
}

// Función que actualiza las probabilidades de los movimientos
vector<float> ActualizarProbabilidades(vector<int> *Numeradores, int *Denominador, int Movimiento){
    
    int i, DenominadorAux = *Denominador;
    float Division;
    vector<int> NumeradoresAux = *Numeradores;
    vector<float> Probabilidades, ProbAcumuladas;

    DenominadorAux++;
    NumeradoresAux[Movimiento]++;

    for(i=0; i<(int)NumeradoresAux.size(); i++){
        Division = (float) NumeradoresAux[i] / DenominadorAux;
        Probabilidades.push_back(Division);
    }

    ProbAcumuladas.push_back(Probabilidades[0]);
    for(i=1; i<(int)Probabilidades.size(); i++){
        ProbAcumuladas.push_back(Probabilidades[i] + ProbAcumuladas[i-1]);
    }

    *Numeradores = NumeradoresAux;
    *Denominador = DenominadorAux;

    return ProbAcumuladas;
}

// *********************************************************************************************************** //
// *********************************************** MOVIMIENTOS *********************************************** //
// *********************************************************************************************************** //

// Movimiento que hace swap entre 2 nodos de rutas distintas de un mismo drone
vector<vector<int>> Swap(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosSwap){

    int NodoOrigen, PrimerNodo, SegundoNodo, DroneElegido, aux;
    NodosSwap->clear();
   
    DroneElegido = int_rand(0, DroneNum);
    vector<int> NuevaRuta = Rutas[DroneElegido];

    // Verificar que la ruta del drone seleccionado tenga más de 1 cliente
    while(true){
        if(NuevaRuta.size()>3)
            break;
        else{
            DroneElegido = int_rand(0, DroneNum);
            NuevaRuta = Rutas[DroneElegido];
        }
    }

    NodosSwap->push_back(DroneElegido);

    // Se selecciona el primer nodo aleatoriamente
    PrimerNodo = int_rand(1, NuevaRuta.size()-1);
    NodoOrigen = NuevaRuta[0];

    // Verificar que el nodo seleccionado no sea el mismo al nodo origen
    while(true){
        if(NuevaRuta[PrimerNodo] != NodoOrigen)
            break;
        else
            PrimerNodo = int_rand(1, NuevaRuta.size()-1);
    }

    SegundoNodo = int_rand(1, NuevaRuta.size()-1);
   
    // Verificar que el nodo seleccionado no sea el mismo al nodo origen ni al nodo seleccionado anteriormente
    while(true){
        if(NuevaRuta[SegundoNodo] != NodoOrigen && SegundoNodo != PrimerNodo)
            break;
        else
            SegundoNodo = int_rand(1, NuevaRuta.size()-1);
    }

    NodosSwap->push_back(PrimerNodo);
    NodosSwap->push_back(SegundoNodo);

    aux = NuevaRuta[PrimerNodo];
    NuevaRuta[PrimerNodo] = NuevaRuta[SegundoNodo];
    NuevaRuta[SegundoNodo] = aux;

    Rutas[DroneElegido] = NuevaRuta;
    return Rutas;
}

// Movimiento que mueve un cliente al azar a una posición al azar dentro del mismo drone
vector<vector<int>> MoverCliente(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosMover){

    int NodoOrigen, NodoFinal, ClienteMovido, PrimerNodo, SegundoNodo, DroneElegido, UnicoCliente = 0;
    NodosMover->clear();
   
    DroneElegido = int_rand(0, DroneNum);

    while(true){
        if(DroneElegido != 0)
            break;
        else
            DroneElegido = int_rand(0, DroneNum);
    }

    vector<int> NuevaRuta = Rutas[DroneElegido];

    // Verificar que la ruta del drone seleccionado tenga más de 1 cliente
    while(true){
        if(NuevaRuta.size()>3)
            break;
        else{
            DroneElegido = int_rand(0, DroneNum);
            while(true){
                if(DroneElegido != 0)
                    break;
                else
                    DroneElegido = int_rand(0, DroneNum);
            }
            NuevaRuta = Rutas[DroneElegido];
        }
    }

    NodosMover->push_back(DroneElegido);

    // Se selecciona el primer nodo aleatoriamente
    PrimerNodo = int_rand(1, NuevaRuta.size()-1);
    NodoOrigen = NuevaRuta[0];
    NodoFinal = Rutas[DroneElegido][NuevaRuta.size()-1];

    // Verificar que el nodo seleccionado no sea el mismo al nodo origen
    while(true){
        if(NuevaRuta[PrimerNodo] != NodoOrigen){
            if(NuevaRuta[PrimerNodo-1] == NodoOrigen && NuevaRuta[PrimerNodo+1] == NodoFinal)
                PrimerNodo = int_rand(1, NuevaRuta.size()-1);
            else
                break;
        }
        else
            PrimerNodo = int_rand(1, NuevaRuta.size()-1);
    }

    ClienteMovido = NuevaRuta[PrimerNodo];

    if(NuevaRuta[PrimerNodo-1] == NodoOrigen && NuevaRuta[PrimerNodo+1] == NodoOrigen){
        NuevaRuta.erase(NuevaRuta.begin()+PrimerNodo);
        UnicoCliente = 1;
    }
    NuevaRuta.erase(NuevaRuta.begin()+PrimerNodo);

    SegundoNodo = int_rand(1, NuevaRuta.size()-1);

    NodosMover->push_back(PrimerNodo);
    NodosMover->push_back(SegundoNodo);
    NodosMover->push_back(UnicoCliente);

    NuevaRuta.insert(NuevaRuta.begin()+SegundoNodo, ClienteMovido);

    Rutas[DroneElegido] = NuevaRuta;
    return Rutas;
}

// Movimiento que mueve un cliente al azar a una posición al azar dentro de un drone distinto
vector<vector<int>> MoverCliente2(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosMover){

    int NodoOrigen, NodoFinal, ClienteMovido, PrimerNodo, SegundoNodo, Drone1, Drone2, UnicoCliente = 0;
    vector<int> Ruta1, Ruta2;
    NodosMover->clear();
   
    Drone1 = int_rand(0, DroneNum);

    while(true){
        if(Drone1 != 0)
            break;
        else
            Drone1 = int_rand(0, DroneNum);
    }

    Ruta1 = Rutas[Drone1];

    // Verificar que la ruta del drone seleccionado tenga más de 1 cliente
    while(true){
        if(Ruta1.size()>3)
            break;
        else{
            Drone1 = int_rand(0, DroneNum);
            while(true){
                if(Drone1 != 0)
                    break;
                else
                    Drone1 = int_rand(0, DroneNum);
            }
            Ruta1 = Rutas[Drone1];
        }
    }

    NodosMover->push_back(Drone1);

    // Se selecciona el primer nodo aleatoriamente
    PrimerNodo = int_rand(1, Ruta1.size()-1);
    NodoOrigen = Ruta1[0];
    NodoFinal = Rutas[Drone1][Ruta1.size()-1];

    // Verificar que el nodo seleccionado no sea el mismo al nodo origen
    while(true){
        if(Ruta1[PrimerNodo] != NodoOrigen){
            if(Ruta1[PrimerNodo-1] == NodoOrigen && Ruta1[PrimerNodo+1] == NodoFinal)
                PrimerNodo = int_rand(1, Ruta1.size()-1);
            else
                break;
        }
        else
            PrimerNodo = int_rand(1, Ruta1.size()-1);
    }

    ClienteMovido = Ruta1[PrimerNodo];

    // Se borra el cliente de la Ruta1
    if(Ruta1[PrimerNodo-1] == NodoOrigen && Ruta1[PrimerNodo+1] == NodoOrigen){
        Ruta1.erase(Ruta1.begin()+PrimerNodo);
        UnicoCliente = 1;
    }
    Ruta1.erase(Ruta1.begin()+PrimerNodo);

    // Se selecciona el segundo drone
    Drone2 = int_rand(0, DroneNum);

    while(true){
        if(Drone2 != Drone1)
            break;
        else
            Drone2 = int_rand(0, DroneNum);
    }

    Ruta2 = Rutas[Drone2];
    SegundoNodo = int_rand(1, Ruta2.size()-1);

    Ruta2.insert(Ruta2.begin()+SegundoNodo, ClienteMovido);

    NodosMover->push_back(Drone2);
    NodosMover->push_back(PrimerNodo);
    NodosMover->push_back(SegundoNodo);
    NodosMover->push_back(UnicoCliente);

    Rutas[Drone1] = Ruta1;
    Rutas[Drone2] = Ruta2;

    return Rutas;
}

// Movimiento que selecciona un cliente al azar y crea una ruta con ese cliente en una posición al azar dentro del mismo drone
vector<vector<int>> CrearRuta(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosCortar){

    int NodoOrigen, NodoFinal, NodoSeleccionado, Posicion, DroneElegido, FlagCortar, UnicoCliente = 0;
    NodosCortar->clear();
   
    DroneElegido = int_rand(0, DroneNum);

    while(true){
        if(DroneElegido != 0)
            break;
        else
            DroneElegido = int_rand(0, DroneNum);
    }

    vector<int> NuevaRuta = Rutas[DroneElegido];
    vector<int> RutaAux = NuevaRuta;
   

    // Verificar que la ruta del drone seleccionado tenga más de 1 cliente
    while(true){
        if(NuevaRuta.size()>3)
            break;
        else{
            DroneElegido = int_rand(0, DroneNum);
            while(true){
                if(DroneElegido != 0)
                    break;
                else
                    DroneElegido = int_rand(0, DroneNum);
            }
            NuevaRuta = Rutas[DroneElegido];
            RutaAux = NuevaRuta;
        }
    }

    NodosCortar->push_back(DroneElegido);

    // Se selecciona el primer nodo aleatoriamente
    NodoSeleccionado = int_rand(1, NuevaRuta.size()-1);
    NodoOrigen = NuevaRuta[0];
    NodoFinal = Rutas[DroneElegido][NuevaRuta.size()-1];

    // Verificar que el nodo seleccionado no sea el mismo al nodo origen
    while(true){
        if(NuevaRuta[NodoSeleccionado] != NodoOrigen){
            if(NuevaRuta[NodoSeleccionado-1] == NodoOrigen && NuevaRuta[NodoSeleccionado+1] == NodoFinal)
                NodoSeleccionado = int_rand(1, NuevaRuta.size()-1);
            else
                break;
        }
        else
            NodoSeleccionado = int_rand(1, NuevaRuta.size()-1);
    }

    NodosCortar->push_back(NodoSeleccionado);

    if(NuevaRuta[NodoSeleccionado-1] == NodoOrigen && NuevaRuta[NodoSeleccionado+1] == NodoOrigen){
        RutaAux.erase(RutaAux.begin()+NodoSeleccionado);
        UnicoCliente = 1;
    }
    RutaAux.erase(RutaAux.begin()+NodoSeleccionado);

    Posicion = int_rand(1, RutaAux.size());

    if(RutaAux[Posicion] != NodoOrigen && RutaAux[Posicion] != NodoFinal){
        if(RutaAux[Posicion-1] != NodoOrigen){
            RutaAux.insert(RutaAux.begin()+Posicion, 0);
            RutaAux.insert(RutaAux.begin()+Posicion+1, NuevaRuta[NodoSeleccionado]);
            RutaAux.insert(RutaAux.begin()+Posicion+2, 0);
            // Se inserta un cero a la izquierda y uno a la derecha
            FlagCortar = 1;
        }
        else{
            RutaAux.insert(RutaAux.begin()+Posicion, NuevaRuta[NodoSeleccionado]);
            RutaAux.insert(RutaAux.begin()+Posicion+1, 0);
            // Se inserta un cero a la derecha
            FlagCortar = 2;
        }
    }
       
    else{
        RutaAux.insert(RutaAux.begin()+Posicion, 0);
        RutaAux.insert(RutaAux.begin()+Posicion+1, NuevaRuta[NodoSeleccionado]);
        // Se inserta un cero a la izquierda
        FlagCortar = 3;
    }

    NodosCortar->push_back(Posicion);
    NodosCortar->push_back(FlagCortar);
    NodosCortar->push_back(UnicoCliente);

    NuevaRuta = RutaAux;
    Rutas[DroneElegido] = NuevaRuta;
    return Rutas;
}

// Movimiento que selecciona un cliente al azar y crea una ruta con ese cliente en una posición al azar dentro de un drone distinto
vector<vector<int>> CrearRuta2(vector<vector<int>> Rutas, int *NumRutas, int NumDrones, vector<int> *NodosCortar){

    int NodoOrigen, NodoFinal, NodoSeleccionado, NumRutasAux = *NumRutas, Posicion, Drone1, Drone2, FlagCortar, UnicoCliente = 0;
    NodosCortar->clear();
    vector<int> Ruta1, Ruta2, RutaAux1, RutaAux2, RutaExtra;
   
    Drone1 = int_rand(0, NumRutasAux);

    while(true){
        if(Drone1 != 0)
            break;
        else
            Drone1 = int_rand(0, NumRutasAux);
    }

    Ruta1 = Rutas[Drone1];
    RutaAux1 = Ruta1;

    // Verificar que la ruta del drone seleccionado tenga más de 1 cliente
    while(true){
        if(Ruta1.size()>3)
            break;
        else{
            Drone1 = int_rand(0, NumRutasAux);
            while(true){
                if(Drone1 != 0)
                    break;
                else
                    Drone1 = int_rand(0, NumRutasAux);
            }
            Ruta1 = Rutas[Drone1];
            RutaAux1 = Ruta1;
        }
    }

    NodosCortar->push_back(Drone1);
    Drone2 = int_rand(0, NumDrones);

    // Verificar que el drone seleccionado no sea el mismo al anterior
    while(true){
        if(Drone2 != Drone1)
            break;
        else
            Drone2 = int_rand(0, NumDrones);
    }

    if(Drone2 >= NumRutasAux){
        Drone2 = NumRutasAux;
        // Se selecciona el primer nodo aleatoriamente
        NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        NodoOrigen = Ruta1[0];
        NodoFinal = Rutas[Drone1][Ruta1.size()-1];

        // Verificar que el nodo seleccionado no sea el mismo al nodo origen
        while(true){
            if(Ruta1[NodoSeleccionado] != NodoOrigen){
                if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoFinal)
                    NodoSeleccionado = int_rand(1, Ruta1.size()-1);
                else
                    break;
            }
            else
                NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        }

        NodosCortar->push_back(Drone2);
        NodosCortar->push_back(NodoSeleccionado);

        if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoOrigen){
            RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);
            UnicoCliente = 1;
        }
        RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);

        RutaExtra.push_back(NodoOrigen);
        RutaExtra.push_back(Ruta1[NodoSeleccionado]);
        RutaExtra.push_back(NodoFinal);
        Rutas.push_back(RutaExtra);
        Ruta1 = RutaAux1;
        Rutas[Drone1] = Ruta1;
        NodosCortar->push_back(1);
        FlagCortar = 4;
        NodosCortar->push_back(FlagCortar);
        NodosCortar->push_back(UnicoCliente);
        NumRutasAux++;
        *NumRutas = NumRutasAux;

    }
    else{
        Ruta2 = Rutas[Drone2];
        RutaAux2 = Ruta2;

        NodosCortar->push_back(Drone2);

        // Se selecciona el primer nodo aleatoriamente
        NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        NodoOrigen = Ruta1[0];
        NodoFinal = Rutas[Drone1][Ruta1.size()-1];

        // Verificar que el nodo seleccionado no sea el mismo al nodo origen
        while(true){
            if(Ruta1[NodoSeleccionado] != NodoOrigen){
                if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoFinal)
                    NodoSeleccionado = int_rand(1, Ruta1.size()-1);
                else
                    break;
            }
            else
                NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        }

        NodosCortar->push_back(NodoSeleccionado);

        if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoOrigen){
            RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);
            UnicoCliente = 1;
        }
        RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);

        Posicion = int_rand(1, Ruta2.size());

        if(RutaAux2[Posicion] != NodoOrigen && RutaAux2[Posicion] != NodoFinal){
            if(RutaAux2[Posicion-1] != NodoOrigen){
                RutaAux2.insert(RutaAux2.begin()+Posicion, 0);
                RutaAux2.insert(RutaAux2.begin()+Posicion+1, Ruta1[NodoSeleccionado]);
                RutaAux2.insert(RutaAux2.begin()+Posicion+2, 0);
                // Se inserta un cero a la izquierda y uno a la derecha
                FlagCortar = 1;
            }
            else{
                RutaAux2.insert(RutaAux2.begin()+Posicion, Ruta1[NodoSeleccionado]);
                RutaAux2.insert(RutaAux2.begin()+Posicion+1, 0);
                // Se inserta un cero a la derecha
                FlagCortar = 2;
            }
        }
           
        else{
            RutaAux2.insert(RutaAux2.begin()+Posicion, 0);
            RutaAux2.insert(RutaAux2.begin()+Posicion+1, Ruta1[NodoSeleccionado]);
            // Se inserta un cero a la izquierda
            FlagCortar = 3;
        }

        NodosCortar->push_back(Posicion);
        NodosCortar->push_back(FlagCortar);
        NodosCortar->push_back(UnicoCliente);

        Ruta1 = RutaAux1;
        Ruta2 = RutaAux2;
        Rutas[Drone1] = Ruta1;
        Rutas[Drone2] = Ruta2;
    }
   
    return Rutas;
}

// Movimiento que mueve una ruta desde un drone ocupado a uno desocupado
vector<vector<int>> BalanceoCargaA1(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosSwap, vector<int> *RutaMovida){
    int i, DroneOcupado, DroneDesocupado, MasVisitados = -1, MenosVisitados = 9999, ClienteSeleccionado, Contador = 0, Reintentos = 1000;
    int Posicion, NodoOrigen, NodoFinal, ValorCliente, InicioRuta, FinRuta, FlagBalanceo, FlagInfactible = 0;

    vector<int> Ruta1, Ruta2, RutaMovidaAux, VisitadosDrones = ClientesDrones(Rutas), NodosMover = *NodosSwap;
    NodosMover.clear();
    RutaMovida->clear();

    // Se encuentra el drone más ocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] > MasVisitados){
            MasVisitados = VisitadosDrones[i];
            DroneOcupado = i;
        }
    }

    // Se encuentra el drone más desocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] < MenosVisitados){
            MenosVisitados = VisitadosDrones[i];
            DroneDesocupado = i;
        }
    }

    if(DroneOcupado == DroneDesocupado){
        DroneOcupado = int_rand(0, DroneNum);
        DroneDesocupado = int_rand(0, DroneNum);

        // Verificar que el drone seleccionado no sea el mismo al anterior
        while(true){
            if(DroneDesocupado != DroneOcupado)
                break;
            else
                DroneDesocupado = int_rand(0, DroneNum);
        }
    }

    if(DroneOcupado != DroneDesocupado){
        Ruta1 = Rutas[DroneOcupado];
        Ruta2 = Rutas[DroneDesocupado];
        NodosMover.push_back(DroneOcupado);
        NodosMover.push_back(DroneDesocupado);

        // Se selecciona un cliente aleatoriamente del drone ocupado
        ClienteSeleccionado = int_rand(1, Ruta1.size()-1);
        NodoOrigen = Ruta1[0];
        NodoFinal = Rutas[DroneOcupado][Ruta1.size()-1];

        // El cliente seleccionado no puede tener al depósito a cada lado
        while(true){

            if(Contador == Reintentos){
                NodosMover.push_back(-1);
                FlagInfactible = 1;
                break;
            }

            if(Ruta1[ClienteSeleccionado] == NodoOrigen)
                ClienteSeleccionado = int_rand(1, Ruta1.size()-1);
            else if(Ruta1[ClienteSeleccionado-1] == NodoOrigen){
                if(Ruta1[ClienteSeleccionado+1] == NodoOrigen || Ruta1[ClienteSeleccionado+1] == NodoFinal){
                    ClienteSeleccionado = int_rand(1, Ruta1.size()-1);
                }
                else
                    break;
            }
            else
                break;

            Contador++;
        }

        if(FlagInfactible != 1){
            // Busco el inicio de la ruta desde el cliente seleccionado
            while(true){
                ValorCliente = Ruta1[ClienteSeleccionado];
                if(ValorCliente == NodoOrigen)
                    break;
                ClienteSeleccionado--;
            }

            InicioRuta = ClienteSeleccionado;
            ClienteSeleccionado++;

            while(true){
                if(Ruta1[ClienteSeleccionado] == NodoOrigen || Ruta1[ClienteSeleccionado] == NodoFinal)
                    break;
                RutaMovida->push_back(Ruta1[ClienteSeleccionado]);
                ClienteSeleccionado++;
            }

            RutaMovidaAux = *RutaMovida;

            for(i=0;i<(int)RutaMovidaAux.size()+1;i++){
                Ruta1.erase(Ruta1.begin()+InicioRuta);
            }

            Posicion = int_rand(1, Ruta2.size()-1);
            FinRuta = Posicion;

            // Se inserta el depósito inicial
            if(Ruta2[Posicion] == NodoOrigen){
                Ruta2.insert(Ruta2.begin()+Posicion, 0);
                FinRuta = Posicion+1;
                Posicion++;
                FlagBalanceo = 1;
            }

            else{
                // Se inserta el depósito final
                if(Ruta2[Posicion-1] == NodoOrigen){
                    Ruta2.insert(Ruta2.begin()+Posicion, 0);
                    FinRuta = Posicion;
                    FlagBalanceo = 1;
                }
                // Se insertan ambos depósitos
                else{
                    Ruta2.insert(Ruta2.begin()+Posicion, 0);
                    Ruta2.insert(Ruta2.begin()+Posicion, 0);
                    FinRuta = Posicion + 1;
                    Posicion++;
                    FlagBalanceo = 2;
                }
            }

            // Se inserta la ruta movida a la posicion seleccionada
            for(i=0;i<(int)RutaMovidaAux.size();i++){
                Ruta2.insert(Ruta2.begin()+FinRuta, RutaMovidaAux[i]);
                FinRuta++;
            }

            InicioRuta++;
            NodosMover.push_back(InicioRuta);
            NodosMover.push_back(FinRuta);
            NodosMover.push_back(Posicion);
            NodosMover.push_back(FlagBalanceo);

            Rutas[DroneOcupado] = Ruta1;
            Rutas[DroneDesocupado] = Ruta2;

        }

       
    }

    *NodosSwap = NodosMover;
    return Rutas;

}

// Movimiento que mueve un cliente desde una ruta ocupada a una ruta desocupada
vector<vector<int>> BalanceoCargaA2(vector<vector<int>> Rutas, int DroneNum, vector<int> *NodosBalanceo){

    int i, NodoOrigen, NodoFinal, ClienteMovido, PrimerNodo, SegundoNodo, Drone1, Drone2, UnicoCliente = 0;
    int MasVisitados = -1, MenosVisitados = 9999;

    vector<int> Ruta1, Ruta2, RutaAux1, RutaAux2, VisitadosDrones = ClientesDrones(Rutas);
    NodosBalanceo->clear();

    // Se encuentra el drone más ocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] > MasVisitados){
            MasVisitados = VisitadosDrones[i];
            Drone1 = i;
        }
    }

    // Se encuentra el drone más desocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] < MenosVisitados){
            MenosVisitados = VisitadosDrones[i];
            Drone2 = i;
        }
    }

    if(Drone1 == Drone2){
        Drone1 = int_rand(0, DroneNum);
        Drone2 = int_rand(0, DroneNum);

        // Verificar que el drone seleccionado no sea el mismo al anterior
        while(true){
            if(Drone2 != Drone1)
                break;
            else
                Drone2 = int_rand(0, DroneNum);
        }
    }

    if(Drone1 != Drone2){
        Ruta1 = Rutas[Drone1];
        Ruta2 = Rutas[Drone2];
        RutaAux1 = Ruta1;
        RutaAux2 = Ruta2;

        NodosBalanceo->push_back(Drone1);
        NodosBalanceo->push_back(Drone2);

        // Se selecciona el primer nodo aleatoriamente
        PrimerNodo = int_rand(1, Ruta1.size()-1);
        NodoOrigen = Ruta1[0];
        NodoFinal = Rutas[Drone1][Ruta1.size()-1];

        // Verificar que el nodo seleccionado no sea el mismo al nodo origen
        while(true){
            if(Ruta1[PrimerNodo] != NodoOrigen){
                if(Ruta1[PrimerNodo-1] == NodoOrigen && Ruta1[PrimerNodo+1] == NodoFinal)
                    PrimerNodo = int_rand(1, Ruta1.size()-1);
                else
                    break;
            }
            else
                PrimerNodo = int_rand(1, Ruta1.size()-1);
        }

        ClienteMovido = Ruta1[PrimerNodo];
        // Se borra el cliente de la Ruta1
        if(Ruta1[PrimerNodo-1] == NodoOrigen && Ruta1[PrimerNodo+1] == NodoOrigen){
            Ruta1.erase(Ruta1.begin()+PrimerNodo);
            UnicoCliente = 1;
        }
        Ruta1.erase(Ruta1.begin()+PrimerNodo);

        SegundoNodo = int_rand(1, Ruta2.size()-1);

        Ruta2.insert(Ruta2.begin()+SegundoNodo, ClienteMovido);

        NodosBalanceo->push_back(PrimerNodo);
        NodosBalanceo->push_back(SegundoNodo);
        NodosBalanceo->push_back(UnicoCliente);

        Rutas[Drone1] = Ruta1;
        Rutas[Drone2] = Ruta2;

    }

    return Rutas;

}

// Movimiento que selecciona un cliente al azar de un drone ocupado y crea una ruta con ese cliente en una posición al azar dentro de un drone desocupado
vector<vector<int>> BalanceoCargaB(vector<vector<int>> Rutas, int NumRutas, int NumDrones, vector<int> *NodosBalanceo){

    int i, NodoOrigen, NodoFinal, NodoSeleccionado, Posicion, Drone1, Drone2, FlagCortar, UnicoCliente = 0;
    int MasVisitados = -1, MenosVisitados = 9999;
    NodosBalanceo->clear();

    vector<int> Ruta1, Ruta2, RutaAux1, RutaAux2, RutaExtra, VisitadosDrones = ClientesDrones(Rutas);

    // Se encuentra el drone más ocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] > MasVisitados){
            MasVisitados = VisitadosDrones[i];
            Drone1 = i;
        }
    }

    // Se encuentra el drone más desocupado
    for(i=0; i<(int)VisitadosDrones.size(); i++){
        if(VisitadosDrones[i] < MenosVisitados){
            MenosVisitados = VisitadosDrones[i];
            Drone2 = i;
        }
    }

    if(Drone1 == Drone2){
        Drone1 = int_rand(0, NumRutas);
        Drone2 = int_rand(0, NumRutas);

        // Verificar que el drone seleccionado no sea el mismo al anterior
        while(true){
            if(Drone2 != Drone1)
                break;
            else
                Drone2 = int_rand(0, NumRutas);
        }
    }

    if(Drone1 != Drone2){
        Ruta1 = Rutas[Drone1];
        Ruta2 = Rutas[Drone2];
        RutaAux1 = Ruta1;
        RutaAux2 = Ruta2;

        NodosBalanceo->push_back(Drone1);
        NodosBalanceo->push_back(Drone2);

        // Se selecciona el primer nodo aleatoriamente
        NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        NodoOrigen = Ruta1[0];
        NodoFinal = Rutas[Drone1][Ruta1.size()-1];

        // Verificar que el nodo seleccionado no sea el mismo al nodo origen
        while(true){
            if(Ruta1[NodoSeleccionado] != NodoOrigen){
                if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoFinal)
                    NodoSeleccionado = int_rand(1, Ruta1.size()-1);
                else
                    break;
            }
            else
                NodoSeleccionado = int_rand(1, Ruta1.size()-1);
        }

        NodosBalanceo->push_back(NodoSeleccionado);

        if(Ruta1[NodoSeleccionado-1] == NodoOrigen && Ruta1[NodoSeleccionado+1] == NodoOrigen){
            RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);
            UnicoCliente = 1;
        }
        RutaAux1.erase(RutaAux1.begin()+NodoSeleccionado);

        Posicion = int_rand(1, Ruta2.size());

        if(RutaAux2[Posicion] != NodoOrigen && RutaAux2[Posicion] != NodoFinal){
            if(RutaAux2[Posicion-1] != NodoOrigen){
                RutaAux2.insert(RutaAux2.begin()+Posicion, 0);
                RutaAux2.insert(RutaAux2.begin()+Posicion+1, Ruta1[NodoSeleccionado]);
                RutaAux2.insert(RutaAux2.begin()+Posicion+2, 0);
                // Se inserta un cero a la izquierda y uno a la derecha
                FlagCortar = 1;
            }
            else{
                RutaAux2.insert(RutaAux2.begin()+Posicion, Ruta1[NodoSeleccionado]);
                RutaAux2.insert(RutaAux2.begin()+Posicion+1, 0);
                // Se inserta un cero a la derecha
                FlagCortar = 2;
            }
        }
           
        else{
            RutaAux2.insert(RutaAux2.begin()+Posicion, 0);
            RutaAux2.insert(RutaAux2.begin()+Posicion+1, Ruta1[NodoSeleccionado]);
            // Se inserta un cero a la izquierda
            FlagCortar = 3;
        }

        NodosBalanceo->push_back(Posicion);
        NodosBalanceo->push_back(FlagCortar);
        NodosBalanceo->push_back(UnicoCliente);

        Ruta1 = RutaAux1;
        Ruta2 = RutaAux2;
        Rutas[Drone1] = Ruta1;
        Rutas[Drone2] = Ruta2;
    }

    return Rutas;

}

// ********************************************************************************************************** //
// ************************************ ACTUALIZAR FUNCIÓN DE EVALUACIÓN ************************************ //
// ********************************************************************************************************** //

// Función que actualiza la función de evaluación luego de aplicarle un movimiento
float ActualizarCalidadSwap(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone = NodosSwap[0];
    int Nodo1 = NodosSwap[1];
    int Nodo2 = NodosSwap[2];

    // Se resta la distancia antigua y se suma la nueva para el Nodo 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1-1], RutasAntiguas[Drone][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1], RutasAntiguas[Drone][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo1-1], RutasNuevas[Drone][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo1], RutasNuevas[Drone][Nodo1+1], X_coor, Y_coor, delta, tipo, q);

    // Se resta la distancia antigua y se suma la nueva para el Nodo 2
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo2-1], RutasAntiguas[Drone][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo2], RutasAntiguas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2-1], RutasNuevas[Drone][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2], RutasNuevas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadMover(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone = NodosSwap[0];
    int Nodo1 = NodosSwap[1];
    int Nodo2 = NodosSwap[2];

    // Se resta la distancia antigua y se suma la nueva para el Nodo 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1-1], RutasAntiguas[Drone][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1], RutasAntiguas[Drone][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
   
    if(Nodo1 < Nodo2)
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo1-1], RutasNuevas[Drone][Nodo1], X_coor, Y_coor, delta, tipo, q);
    else if(Nodo1 > Nodo2)
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo1], RutasNuevas[Drone][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
   
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2-1], RutasNuevas[Drone][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2], RutasNuevas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadMover2(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int Nodo1 = NodosSwap[2];
    int Nodo2 = NodosSwap[3];

    // Ruta 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1], RutasAntiguas[Drone1][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone1][Nodo1-1], RutasNuevas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);

    // Ruta 2
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone2][Nodo2-1], RutasAntiguas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadCrear(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone = NodosSwap[0];
    int Nodo1 = NodosSwap[1];
    int Nodo2 = NodosSwap[2];
    int FlagCortar = NodosSwap[3];

    // Se resta la distancia antigua y se suma la nueva para el Nodo 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1-1], RutasAntiguas[Drone][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone][Nodo1], RutasAntiguas[Drone][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    if(FlagCortar == 1){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2-1], RutasNuevas[Drone][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2], RutasNuevas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2+1], RutasNuevas[Drone][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2+2], RutasNuevas[Drone][Nodo2+3], X_coor, Y_coor, delta, tipo, q);
    }
    else if(FlagCortar == 2){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2-1], RutasNuevas[Drone][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2], RutasNuevas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
    }

    else if(FlagCortar == 3){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2], RutasNuevas[Drone][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone][Nodo2+1], RutasNuevas[Drone][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
    }

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadCrear2(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int Nodo1 = NodosSwap[2];
    int Nodo2 = NodosSwap[3];
    int FlagCortar = NodosSwap[4];

    // Se resta la distancia antigua y se suma la nueva para el Nodo 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1], RutasAntiguas[Drone1][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    if(FlagCortar == 1){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+1], RutasNuevas[Drone2][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+2], RutasNuevas[Drone2][Nodo2+3], X_coor, Y_coor, delta, tipo, q);
    }
    else if(FlagCortar == 2 || FlagCortar == 4){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
    }

    else if(FlagCortar == 3){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+1], RutasNuevas[Drone2][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
    }

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadBalanceoCargaA1(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    //int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    //int InicioRuta = NodosSwap[2];
    int FinRuta = NodosSwap[3];
    int Posicion = NodosSwap[4];
    int FlagBalanceo = NodosSwap[5];

    // Si se inserta un depósito inicial o un depósito final (FlagBalanceo = 1), no se actualiza la función de evaluación

    // Se inserta un depósito inicial y final
    if(FlagBalanceo == 2){
        FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone2][Posicion-2], RutasAntiguas[Drone2][Posicion-1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Posicion-2], RutasNuevas[Drone2][Posicion-1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][FinRuta-1], RutasNuevas[Drone2][FinRuta+1], X_coor, Y_coor, delta, tipo, q);
    }
    return FuncionEvaluacionTotal;
}

float ActualizarCalidadBalanceoCargaA2(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int Nodo1 = NodosSwap[2];
    int Nodo2 = NodosSwap[3];

    // Ruta 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1], RutasAntiguas[Drone1][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone1][Nodo1-1], RutasNuevas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);

    // Ruta 2
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone2][Nodo2-1], RutasAntiguas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);

    return FuncionEvaluacionTotal;

}

float ActualizarCalidadBalanceoCargaB(vector<vector<int>> RutasAntiguas, vector<vector<int>> RutasNuevas, vector<int> NodosSwap,
float FuncionEvaluacionTotal, vector<int> X_coor, vector<int> Y_coor, int delta, int tipo, float q){

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int Nodo1 = NodosSwap[2];
    int Nodo2 = NodosSwap[3];
    int FlagCortar = NodosSwap[4];

    // Se resta la distancia antigua y se suma la nueva para el Nodo 1
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal -= FuncionEvaluacionNodos(RutasAntiguas[Drone1][Nodo1], RutasAntiguas[Drone1][Nodo1+1], X_coor, Y_coor, delta, tipo, q);
    FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone1][Nodo1-1], RutasAntiguas[Drone1][Nodo1], X_coor, Y_coor, delta, tipo, q);
    if(FlagCortar == 1){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+1], RutasNuevas[Drone2][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+2], RutasNuevas[Drone2][Nodo2+3], X_coor, Y_coor, delta, tipo, q);
    }
    else if(FlagCortar == 2 || FlagCortar == 4){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2-1], RutasNuevas[Drone2][Nodo2], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
    }

    else if(FlagCortar == 3){
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2], RutasNuevas[Drone2][Nodo2+1], X_coor, Y_coor, delta, tipo, q);
        FuncionEvaluacionTotal += FuncionEvaluacionNodos(RutasNuevas[Drone2][Nodo2+1], RutasNuevas[Drone2][Nodo2+2], X_coor, Y_coor, delta, tipo, q);
    }

    return FuncionEvaluacionTotal;

}

// ************************************************************************************************************ //
// *********************************************** RESTRICCIONES ********************************************** //
// ************************************************************************************************************ //

// Función que verifica las restricciones de la ventana de tiempo, de la energía y del peso
bool RestriccionesSwap(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosSwap, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, NodoMenor, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone = NodosSwap[0];
    int Nodo1 = NodosSwap[1];
    int Nodo2 = NodosSwap[2];

    if(Nodo1 < Nodo2)
        NodoMenor = Nodo1;
    else
        NodoMenor = Nodo2;

    Largo = Rutas[Drone].size();
    DepositoOrigen = Rutas[Drone][0];
    DepositoFinal = Rutas[Drone][Largo-1];

    DistanciaActual = Tiempos[Drone][NodoMenor-1];
    EnergiaActual = Energias[Drone][NodoMenor-1];
    PesoActual = Pesos[Drone][NodoMenor-1];
    if(Rutas[Drone][NodoMenor-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=NodoMenor;i<Largo;i++){

        if(Rutas[Drone][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            Pesos[Drone][i] = 0;
            break;
        }
        if(Rutas[Drone][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone][i]] && DistanciaActual <= DueTime[Rutas[Drone][i]])
                Tiempos[Drone][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone][i]];
                Tiempos[Drone][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone][i] = PesoActual;
                }
                else{
                    return false;
                }
            }
            else{
                return false;
            }
        }
    }

    return true;
}

// Función que verifica las restricciones de la ventana de tiempo, de la energía y del peso
bool RestriccionesMover(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosMover, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, NodoMenor, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone = NodosMover[0];
    int NodoSeleccionado = NodosMover[1];
    int PosicionDestino = NodosMover[2];
    int UnicoCliente = NodosMover[3];

    if(UnicoCliente == 1){
        Tiempos[Drone].erase(Tiempos[Drone].begin()+NodoSeleccionado);
        Energias[Drone].erase(Energias[Drone].begin()+NodoSeleccionado);
        Pesos[Drone].erase(Pesos[Drone].begin()+NodoSeleccionado);
    }

    if(NodoSeleccionado < PosicionDestino)
        NodoMenor = NodoSeleccionado;
    else
        NodoMenor = PosicionDestino;
   
    Largo = Rutas[Drone].size();
    DepositoOrigen = Rutas[Drone][0];
    DepositoFinal = Rutas[Drone][Largo-1];

    DistanciaActual = Tiempos[Drone][NodoMenor-1];
    EnergiaActual = Energias[Drone][NodoMenor-1];
    PesoActual = Pesos[Drone][NodoMenor-1];
    if(Rutas[Drone][NodoMenor-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;
    for(i=NodoMenor;i<Largo;i++){
       
        if(Rutas[Drone][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            Pesos[Drone][i] = 0;
            break;
        }
       
        if(Rutas[Drone][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone][i]] && DistanciaActual <= DueTime[Rutas[Drone][i]])
                Tiempos[Drone][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone][i]];
                Tiempos[Drone][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    return true;
}

bool RestriccionesMover2(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosMover, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone1 = NodosMover[0];
    int Drone2 = NodosMover[1];
    int NodoSeleccionado = NodosMover[2];
    int PosicionDestino = NodosMover[3];
    int UnicoCliente = NodosMover[4];

    Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
    Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
    Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    if(UnicoCliente == 1){
        Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
        Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
        Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    }
    Tiempos[Drone2].insert(Tiempos[Drone2].begin()+PosicionDestino, 0);
    Energias[Drone2].insert(Energias[Drone2].begin()+PosicionDestino, 0);
    Pesos[Drone2].insert(Pesos[Drone2].begin()+PosicionDestino, 0);

    // Ruta 1
    Largo = Rutas[Drone1].size();
    DepositoOrigen = Rutas[Drone1][0];
    DepositoFinal = Rutas[Drone1][Largo-1];

    DistanciaActual = Tiempos[Drone1][NodoSeleccionado-1];
    EnergiaActual = Energias[Drone1][NodoSeleccionado-1];
    PesoActual = Pesos[Drone1][NodoSeleccionado-1];
    if(Rutas[Drone1][NodoSeleccionado-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;
    for(i=NodoSeleccionado;i<Largo;i++){
       
        if(Rutas[Drone1][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            Pesos[Drone1][i] = 0;
            break;
        }
       
        if(Rutas[Drone1][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone1][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone1][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone1][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone1][i]] && DistanciaActual <= DueTime[Rutas[Drone1][i]])
                Tiempos[Drone1][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone1][i]];
                Tiempos[Drone1][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone1][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone1][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone1][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    // Ruta 2
    Largo = Rutas[Drone2].size();
    DepositoOrigen = Rutas[Drone2][0];
    DepositoFinal = Rutas[Drone2][Largo-1];

    DistanciaActual = Tiempos[Drone2][PosicionDestino-1];
    EnergiaActual = Energias[Drone2][PosicionDestino-1];
    PesoActual = Pesos[Drone2][PosicionDestino-1];
    if(Rutas[Drone2][PosicionDestino-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;
    for(i=PosicionDestino;i<Largo;i++){
       
        if(Rutas[Drone2][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            Pesos[Drone2][i] = 0;
            break;
        }
       
        if(Rutas[Drone2][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone2][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone2][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone2][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone2][i]] && DistanciaActual <= DueTime[Rutas[Drone2][i]])
                Tiempos[Drone2][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone2][i]];
                Tiempos[Drone2][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone2][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone2][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone2][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    return true;
}

bool RestriccionesCrear(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosSwap, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, NodoMenor, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone = NodosSwap[0];
    int NodoSeleccionado = NodosSwap[1];
    int Posicion = NodosSwap[2];
    int FlagCortar = NodosSwap[3];
    int UnicoCliente = NodosSwap[4];

    if(NodoSeleccionado < Posicion)
        NodoMenor = NodoSeleccionado;
    else
        NodoMenor = Posicion;
   
    if(UnicoCliente == 1){
        Tiempos[Drone].erase(Tiempos[Drone].begin()+NodoSeleccionado);
        Energias[Drone].erase(Energias[Drone].begin()+NodoSeleccionado);
        Pesos[Drone].erase(Pesos[Drone].begin()+NodoSeleccionado);
    }

    // Inserta depósitos a ambos lados del cliente seleccionado
    if(FlagCortar == 1){
        Tiempos[Drone].insert(Tiempos[Drone].begin()+Posicion, 0);
        Tiempos[Drone].insert(Tiempos[Drone].begin()+Posicion+1, 0);
        Energias[Drone].insert(Energias[Drone].begin()+Posicion, 0);
        Energias[Drone].insert(Energias[Drone].begin()+Posicion+1, 0);
        Pesos[Drone].insert(Pesos[Drone].begin()+Posicion, 0);
        Pesos[Drone].insert(Pesos[Drone].begin()+Posicion+1, 0);
    }

    // Inserta un depósito a un lado del cliente seleccionado
    else if(FlagCortar == 2 || FlagCortar == 3){
        Tiempos[Drone].insert(Tiempos[Drone].begin()+Posicion, 0);
        Energias[Drone].insert(Energias[Drone].begin()+Posicion, 0);
        Pesos[Drone].insert(Pesos[Drone].begin()+Posicion, 0);
    }

    Largo = Rutas[Drone].size();
    DepositoOrigen = Rutas[Drone][0];
    DepositoFinal = Rutas[Drone][Largo-1];

    DistanciaActual = Tiempos[Drone][NodoMenor-1];
    EnergiaActual = Energias[Drone][NodoMenor-1];
    PesoActual = Pesos[Drone][NodoMenor-1];
    if(Rutas[Drone][NodoMenor-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=NodoMenor;i<Largo;i++){
       
        if(Rutas[Drone][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            Pesos[Drone][i] = 0;
            break;
        }
       
        if(Rutas[Drone][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone][i-1], Rutas[Drone][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone][i]] && DistanciaActual <= DueTime[Rutas[Drone][i]])
                Tiempos[Drone][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone][i]];
                Tiempos[Drone][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    return true;
}

bool RestriccionesCrear2(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosSwap, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;
    vector<float> TiemposExtra, EnergiasExtra, PesosExtra;

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int NodoSeleccionado = NodosSwap[2];
    int Posicion = NodosSwap[3];
    int FlagCortar = NodosSwap[4];
    int UnicoCliente = NodosSwap[5];

    if(UnicoCliente == 1){
        Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
        Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
        Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    }

    // Inserta ceros a ambos lados del cliente seleccionado
    if(FlagCortar == 1){
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+1, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+2, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+1, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+2, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+1, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+2, 0);
    }

    // Inserta un cero al lado derecho o izquierdo del cliente seleccionado
    else if(FlagCortar == 2 || FlagCortar == 3){
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+1, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+1, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+1, 0);
    }

    else if(FlagCortar == 4){
        TiemposExtra.push_back(0);
        TiemposExtra.push_back(0);
        TiemposExtra.push_back(0);
        EnergiasExtra.push_back(0);
        EnergiasExtra.push_back(0);
        EnergiasExtra.push_back(0);
        PesosExtra.push_back(Capacidad);
        PesosExtra.push_back(0);
        PesosExtra.push_back(0);
        Tiempos.push_back(TiemposExtra);
        Energias.push_back(EnergiasExtra);
        Pesos.push_back(PesosExtra);
    }

    // Restricciones para Ruta2

    Largo = Rutas[Drone2].size();
    DepositoOrigen = Rutas[Drone2][0];
    DepositoFinal = Rutas[Drone2][Largo-1];

    DistanciaActual = Tiempos[Drone2][Posicion-1];
    EnergiaActual = Energias[Drone2][Posicion-1];
    PesoActual = Pesos[Drone2][Posicion-1];

    if(Rutas[Drone2][Posicion-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=Posicion;i<Largo;i++){
       
        if(Rutas[Drone2][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            Pesos[Drone2][i] = 0;
            break;
        }
       
        if(Rutas[Drone2][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone2][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone2][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone2][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone2][i]] && DistanciaActual <= DueTime[Rutas[Drone2][i]])
                Tiempos[Drone2][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone2][i]];
                Tiempos[Drone2][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone2][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone2][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone2][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    // Restricciones para Ruta1

    Largo = Rutas[Drone1].size();
    DistanciaActual = Tiempos[Drone1][NodoSeleccionado-1];
    EnergiaActual = Energias[Drone1][NodoSeleccionado-1];
    PesoActual = Pesos[Drone1][NodoSeleccionado-1];
    if(Rutas[Drone1][NodoSeleccionado-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
    Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
    Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);

    for(i=NodoSeleccionado;i<Largo;i++){
       
        if(Rutas[Drone1][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            Pesos[Drone1][i] = 0;
            break;
        }
       
        if(Rutas[Drone1][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone1][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone1][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone1][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone1][i]] && DistanciaActual <= DueTime[Rutas[Drone1][i]])
                Tiempos[Drone1][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone1][i]];
                Tiempos[Drone1][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[i-1], Y_coor[i-1], X_coor[i], Y_coor[i], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone1][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone1][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone1][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    return true;
}

bool RestriccionesBalanceoCargaA1(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosMover, vector<int> RutaMovida, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, DepositoOrigen, DepositoFinal, Largo, LargoRutaMovida;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone1 = NodosMover[0];
    int Drone2 = NodosMover[1];
    int InicioRuta = NodosMover[2];
    //int FinRuta = NodosMover[3];
    int Posicion = NodosMover[4];
    int FlagBalanceo = NodosMover[5];
    LargoRutaMovida = RutaMovida.size();

    // Borrar los valores de la Ruta 1 (ocupada)
    for(i=0; i<=LargoRutaMovida; i++){
        Tiempos[Drone1].erase(Tiempos[Drone1].begin()+InicioRuta);
        Energias[Drone1].erase(Energias[Drone1].begin()+InicioRuta);
        Pesos[Drone1].erase(Pesos[Drone1].begin()+InicioRuta);
    }

    // Insertar los valores de la Ruta 2 (desocupada)

    // Si se inserta un solo depósito (inicial o final)
    if(FlagBalanceo == 1){
        for(i=0; i<=LargoRutaMovida; i++){
            Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
            Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
            Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        }
    }

    // Si se insertan ambos depósitos (inicial y final)
    else if(FlagBalanceo == 2){
        for(i=0; i<=LargoRutaMovida+1; i++){
            Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
            Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
            Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        }
    }

    // Ruta 1
    Largo = Rutas[Drone1].size();
    DepositoOrigen = Rutas[Drone1][0];
    DepositoFinal = Rutas[Drone1][Largo-1];

    DistanciaActual = Tiempos[Drone1][DepositoOrigen];
    EnergiaActual = Energias[Drone1][DepositoOrigen];
    PesoActual = Pesos[Drone1][DepositoOrigen];
    EnergiaActual = 0;
    PesoAcumulado = 0;
    for(i=1;i<Largo;i++){
       
        if(Rutas[Drone1][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            Pesos[Drone1][i] = 0;
            break;
        }
       
        if(Rutas[Drone1][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone1][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone1][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone1][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone1][i]] && DistanciaActual <= DueTime[Rutas[Drone1][i]])
                Tiempos[Drone1][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone1][i]];
                Tiempos[Drone1][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone1][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone1][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone1][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    // Ruta 2
    Largo = Rutas[Drone2].size();
    DepositoOrigen = Rutas[Drone2][0];
    DepositoFinal = Rutas[Drone2][Largo-1];

    DistanciaActual = Tiempos[Drone2][Posicion-1];
    EnergiaActual = Energias[Drone2][Posicion-1];
    PesoActual = Pesos[Drone2][Posicion-1];
    if(Rutas[Drone2][Posicion-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;
    for(i=Posicion;i<Largo;i++){
       
        if(Rutas[Drone2][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            Pesos[Drone2][i] = 0;
            break;
        }
       
        if(Rutas[Drone2][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone2][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone2][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone2][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone2][i]] && DistanciaActual <= DueTime[Rutas[Drone2][i]])
                Tiempos[Drone2][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone2][i]];
                Tiempos[Drone2][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone2][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone2][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone2][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    return true;
}

bool RestriccionesBalanceoCargaA2(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosMover, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;

    int Drone1 = NodosMover[0];
    int Drone2 = NodosMover[1];
    int NodoSeleccionado = NodosMover[2];
    int PosicionDestino = NodosMover[3];
    int UnicoCliente = NodosMover[4];

    Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
    Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
    Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    if(UnicoCliente == 1){
        Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
        Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
        Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    }
    Tiempos[Drone2].insert(Tiempos[Drone2].begin()+PosicionDestino, 0);
    Energias[Drone2].insert(Energias[Drone2].begin()+PosicionDestino, 0);
    Pesos[Drone2].insert(Pesos[Drone2].begin()+PosicionDestino, 0);

    // Ruta 1
    Largo = Rutas[Drone1].size();
    DepositoOrigen = Rutas[Drone1][0];
    DepositoFinal = Rutas[Drone1][Largo-1];

    DistanciaActual = Tiempos[Drone1][NodoSeleccionado-1];
    EnergiaActual = Energias[Drone1][NodoSeleccionado-1];
    PesoActual = Pesos[Drone1][NodoSeleccionado-1];
    if(Rutas[Drone1][NodoSeleccionado-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=NodoSeleccionado;i<Largo;i++){
       
        if(Rutas[Drone1][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            Pesos[Drone1][i] = 0;
            break;
        }
       
        if(Rutas[Drone1][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone1][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone1][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone1][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone1][i]] && DistanciaActual <= DueTime[Rutas[Drone1][i]])
                Tiempos[Drone1][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone1][i]];
                Tiempos[Drone1][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone1][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone1][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone1][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    // Ruta 2
    Largo = Rutas[Drone2].size();
    DepositoOrigen = Rutas[Drone2][0];
    DepositoFinal = Rutas[Drone2][Largo-1];

    DistanciaActual = Tiempos[Drone2][PosicionDestino-1];
    EnergiaActual = Energias[Drone2][PosicionDestino-1];
    PesoActual = Pesos[Drone2][PosicionDestino-1];
    if(Rutas[Drone2][PosicionDestino-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;
    for(i=PosicionDestino;i<Largo;i++){
       
        if(Rutas[Drone2][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            Pesos[Drone2][i] = 0;
            break;
        }
       
        if(Rutas[Drone2][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone2][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone2][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone2][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone2][i]] && DistanciaActual <= DueTime[Rutas[Drone2][i]])
                Tiempos[Drone2][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone2][i]];
                Tiempos[Drone2][i] = DistanciaActual;
            }
           
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone2][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone2][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone2][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }
    return true;
}

bool RestriccionesBalanceoCargaB(vector<vector<int>> Rutas, vector<vector<float>> &Tiempos, vector<vector<float>> &Energias,
vector<vector<float>> &Pesos, vector<int> NodosSwap, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime,
vector<int> DueTime, vector<float> Demand, float Capacidad, float CapacidadEnergetica, int delta, int tipo){

    int i, DepositoOrigen, DepositoFinal, Largo;
    float DistanciaActual, EnergiaActual, PesoActual, PesoAcumulado = 0;
    vector<float> TiemposExtra, EnergiasExtra, PesosExtra;

    int Drone1 = NodosSwap[0];
    int Drone2 = NodosSwap[1];
    int NodoSeleccionado = NodosSwap[2];
    int Posicion = NodosSwap[3];
    int FlagCortar = NodosSwap[4];
    int UnicoCliente = NodosSwap[5];

    Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
    Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
    Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);

    if(UnicoCliente == 1){
        Tiempos[Drone1].erase(Tiempos[Drone1].begin()+NodoSeleccionado);
        Energias[Drone1].erase(Energias[Drone1].begin()+NodoSeleccionado);
        Pesos[Drone1].erase(Pesos[Drone1].begin()+NodoSeleccionado);
    }

    // Inserta ceros a ambos lados del cliente seleccionado
    if(FlagCortar == 1){
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+1, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+2, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+1, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+2, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+1, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+2, 0);
    }

    // Inserta un cero al lado derecho o izquierdo del cliente seleccionado
    else if(FlagCortar == 2 || FlagCortar == 3){
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion, 0);
        Tiempos[Drone2].insert(Tiempos[Drone2].begin()+Posicion+1, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion, 0);
        Energias[Drone2].insert(Energias[Drone2].begin()+Posicion+1, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion, 0);
        Pesos[Drone2].insert(Pesos[Drone2].begin()+Posicion+1, 0);
    }

    else if(FlagCortar == 4){
        TiemposExtra.push_back(0);
        TiemposExtra.push_back(0);
        TiemposExtra.push_back(0);
        EnergiasExtra.push_back(0);
        EnergiasExtra.push_back(0);
        EnergiasExtra.push_back(0);
        PesosExtra.push_back(Capacidad);
        PesosExtra.push_back(0);
        PesosExtra.push_back(0);
        Tiempos.push_back(TiemposExtra);
        Energias.push_back(EnergiasExtra);
        Pesos.push_back(PesosExtra);
    }

    // Restricciones para Ruta2

    Largo = Rutas[Drone2].size();
    DepositoOrigen = Rutas[Drone2][0];
    DepositoFinal = Rutas[Drone2][Largo-1];

    DistanciaActual = Tiempos[Drone2][Posicion-1];
    EnergiaActual = Energias[Drone2][Posicion-1];
    PesoActual = Pesos[Drone2][Posicion-1];

    if(Rutas[Drone2][Posicion-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=Posicion;i<Largo;i++){
       
        if(Rutas[Drone2][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            Pesos[Drone2][i] = 0;
            break;
        }
       
        if(Rutas[Drone2][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone2][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone2][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone2][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone2][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone2][i-1], Rutas[Drone2][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone2][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone2][i]] && DistanciaActual <= DueTime[Rutas[Drone2][i]])
                Tiempos[Drone2][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone2][i]];
                Tiempos[Drone2][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone2][i-1]], Y_coor[Rutas[Drone2][i-1]], X_coor[Rutas[Drone2][i]], Y_coor[Rutas[Drone2][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone2][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone2][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone2][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }

    // Restricciones para Ruta1

    Largo = Rutas[Drone1].size();
    DistanciaActual = Tiempos[Drone1][NodoSeleccionado-1];
    EnergiaActual = Energias[Drone1][NodoSeleccionado-1];
    PesoActual = Pesos[Drone1][NodoSeleccionado-1];
    if(Rutas[Drone1][NodoSeleccionado-1] == DepositoOrigen)
        EnergiaActual = 0;
    PesoAcumulado = Capacidad - PesoActual;

    for(i=NodoSeleccionado;i<Largo;i++){
       
        if(Rutas[Drone1][i] == DepositoFinal){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            Pesos[Drone1][i] = 0;
            break;
        }
       
        if(Rutas[Drone1][i] == DepositoOrigen){
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);
            Tiempos[Drone1][i] = DistanciaActual;
            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], 0);
            if(EnergiaActual > CapacidadEnergetica)
                return false;
            Energias[Drone1][i] = EnergiaActual;
            EnergiaActual = 0;
            PesoActual = Capacidad;
            Pesos[Drone1][i] = PesoActual;
            PesoAcumulado = 0;
        }

        else{
            PesoActual -= Demand[Rutas[Drone1][i]];
            DistanciaActual += FuncionEvaluacionNodos(Rutas[Drone1][i-1], Rutas[Drone1][i], X_coor, Y_coor, delta, tipo, PesoActual);

            if(DistanciaActual > DueTime[Rutas[Drone1][i]]){
                return false;
            }
            if(DistanciaActual >= ReadyTime[Rutas[Drone1][i]] && DistanciaActual <= DueTime[Rutas[Drone1][i]])
                Tiempos[Drone1][i] = DistanciaActual;
            else{
                DistanciaActual = ReadyTime[Rutas[Drone1][i]];
                Tiempos[Drone1][i] = DistanciaActual;
            }

            EnergiaActual += ConsumoEnergetico(X_coor[Rutas[Drone1][i-1]], Y_coor[Rutas[Drone1][i-1]], X_coor[Rutas[Drone1][i]], Y_coor[Rutas[Drone1][i]], PesoActual);
            // Restricción energía
            if(EnergiaActual <= CapacidadEnergetica){
                Energias[Drone1][i] = EnergiaActual;
                PesoAcumulado += Demand[Rutas[Drone1][i]];
                // Restricción peso
                if(PesoAcumulado <= Capacidad){
                    if(PesoActual < 0.1)
                        PesoActual = 0;
                    Pesos[Drone1][i] = PesoActual;
                }
                else{
                    return false;
                }
                   
            }
            else{
                return false;
            }
               
        }
    }
    return true;
}

vector<vector<int>> Perturbacion(vector<vector<int>> Rutas, vector<vector<float>> *Tiempos,
vector<vector<float>> *Energias, vector<vector<float>> *Pesos, int *IteracionesTotales, vector<int> *EstadisticasPerturbacion, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime,
vector<float> Demand, float Capacidad, float CapacidadEnergetica, float *FuncionEvaluacion, int NumRutas, int NumDrones, int delta, int tipo, float q, int CantidadMovimientos, vector<float> *Temperaturas, vector<float> *Calidades, vector<int> *Evaluaciones){

    int Movimiento, DenominadorExploracion = 3, ContadorIteraciones = 0, ContadorAceptacion = 0, IterTotal = *IteracionesTotales;
    float NewFuncionEvaluacion = 999999, Random, T_actual = 0;
    bool Factible = false;

    vector<vector<int>> RutasOperador;
    vector<vector<float>> NewTiempos = *Tiempos;
    vector<vector<float>> NewEnergias = *Energias;
    vector<vector<float>> NewPesos = *Pesos;
    vector<vector<float>> TiemposActuales, EnergiasActuales, PesosActuales;
    vector<int> NodosSwap, RutaMovida, NumeradoresExploracion, Estadisticas_Perturbacion = *EstadisticasPerturbacion, Evaluaciones_Pert = *Evaluaciones;
    vector<float> ProbAcumuladasExploracion, ProbAcumuladasExplotacion, Temperaturas_Pert = *Temperaturas, Calidades_Pert = *Calidades;
   
    // Inicialización probabilidades acumuladas de los movimientos

    // Fase exploración
    ProbAcumuladasExploracion.push_back(0.33);
    ProbAcumuladasExploracion.push_back(0.66);
    ProbAcumuladasExploracion.push_back(1.0);

    NumeradoresExploracion.push_back(1);
    NumeradoresExploracion.push_back(1);
    NumeradoresExploracion.push_back(1);


    while(ContadorAceptacion < CantidadMovimientos){

        IterTotal++;

        if(ContadorIteraciones == 5000)
            break;

        // Se selecciona un número flotante al azar entre 0 y 1
        Random = float_rand(0,1);
        Movimiento = RuletaMovimientos(ProbAcumuladasExploracion, Random);

        // Movimiento: BalanceoCargaA1
        if(Movimiento == 0){
            Estadisticas_Perturbacion[1]++;
            RutasOperador = BalanceoCargaA1(Rutas, NumRutas, &NodosSwap, &RutaMovida);
            TiemposActuales = NewTiempos;
            EnergiasActuales = NewEnergias;
            PesosActuales = NewPesos;
            if(NodosSwap[2] != -1){
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_Pert.push_back(T_actual);
                Calidades_Pert.push_back(NewFuncionEvaluacion);
                Evaluaciones_Pert.push_back(IterTotal);
                Factible = RestriccionesBalanceoCargaA1(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, RutaMovida, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);
            }
           
            else{
                Factible = false;
            }

            if(Factible){
                ContadorAceptacion++;
                Estadisticas_Perturbacion[0]++;
                *FuncionEvaluacion = NewFuncionEvaluacion;
                ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                Rutas = RutasOperador;
            }
            else{
                NewTiempos = TiemposActuales;
                NewEnergias = EnergiasActuales;
                NewPesos = PesosActuales;
            }
        }

        // Movimiento: BalanceoCargaA2
        else if(Movimiento == 1){
            Estadisticas_Perturbacion[3]++;
            RutasOperador = BalanceoCargaA2(Rutas, NumRutas, &NodosSwap);
            NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
            Temperaturas_Pert.push_back(T_actual);
            Calidades_Pert.push_back(NewFuncionEvaluacion);
            Evaluaciones_Pert.push_back(IterTotal);
            TiemposActuales = NewTiempos;
            EnergiasActuales = NewEnergias;
            PesosActuales = NewPesos;
            Factible = RestriccionesBalanceoCargaA2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

            if(Factible){
                ContadorAceptacion++;
                Estadisticas_Perturbacion[2]++;
                *FuncionEvaluacion = NewFuncionEvaluacion;
                ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                Rutas = RutasOperador;
            }

            else{
                NewTiempos = TiemposActuales;
                NewEnergias = EnergiasActuales;
                NewPesos = PesosActuales;
            }
        }

        // Movimiento: BalanceoCargaB
        else if(Movimiento == 2){
            Estadisticas_Perturbacion[5]++;
            RutasOperador = BalanceoCargaB(Rutas, NumRutas, NumDrones, &NodosSwap);
            NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
            Temperaturas_Pert.push_back(T_actual);
            Calidades_Pert.push_back(NewFuncionEvaluacion);
            Evaluaciones_Pert.push_back(IterTotal);
            TiemposActuales = NewTiempos;
            EnergiasActuales = NewEnergias;
            PesosActuales = NewPesos;
            Factible = RestriccionesBalanceoCargaB(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

            if(Factible){
                ContadorAceptacion++;
                Estadisticas_Perturbacion[4]++;
                *FuncionEvaluacion = NewFuncionEvaluacion;
                ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                Rutas = RutasOperador;
            }

            else{
                NewTiempos = TiemposActuales;
                NewEnergias = EnergiasActuales;
                NewPesos = PesosActuales;
            }
        }
       
        ContadorIteraciones++;
    }

    // Falta actualizar los vectores de las estadísticas de los movimientos

    *FuncionEvaluacion = NewFuncionEvaluacion;
    *Tiempos = NewTiempos;
    *Energias = NewEnergias;
    *Pesos = NewPesos;
    *IteracionesTotales = IterTotal;
    *EstadisticasPerturbacion = Estadisticas_Perturbacion;
    *Temperaturas = Temperaturas_Pert;
    *Calidades = Calidades_Pert;
    *Evaluaciones = Evaluaciones_Pert;

    return Rutas;

}

// ************************************************************************************************************ //
// ************************************************* TÉCNICAS ************************************************* //
// ************************************************************************************************************ //

vector<vector<int>> Greedy(vector<int> NodeID, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime,
vector<float> Demand, vector<vector<float>> *Tiempos, vector<vector<float>> *TiemposEspera, vector<vector<float>> *Energias,
vector<vector<float>> *Pesos, int CustNum, int DroneNum, float Capacidad, float CapacidadEnergetica, int *NumRutas){
    int i, j, NodoActual, NodoAnterior, MasTemprano, DroneActual = 0, Fase = 1, LargoNodosPesados, PosicionPesada;
    int Deposito, DepositoFinal, LimiteInferior, LimiteSuperior, DronesUsados = 0;
    float Distancia, Diferencia, TiempoAcumulado = 0, CapacidadAcumulada = 0, EnergiaAcumulada = 0, PesoActual = Capacidad;
    float Q3, LimiteDemanda, TiempoActual = 0, EnergiaActual = 0, Energia, PesoCarga = 1.5, EnergiaDeposito;
    bool RestriccionTiempo = false, HaciaDeposito = false;
    vector <int> Ruta, NoVisitados, Visitados, NodosPesados, ClientesIniciales, Insertados;
    vector<vector<int>> Rutas;
    vector<float> Tiempos_Nodos, Tiempos_Espera, Energia_Nodos, Pesos_Nodos;
    int Largo = X_coor.size()-1;

    Deposito = NodeID[0];
    DepositoFinal = NodeID[CustNum+1];
    NodoActual = NodeID[0];
    NodoAnterior = NodoActual;
    Ruta.push_back(NodoActual);
    Tiempos_Nodos.push_back(TiempoAcumulado);
    Energia_Nodos.push_back(EnergiaAcumulada);
    Pesos_Nodos.push_back(PesoActual);

    // Creo un vector lleno de ceros para representar que inicialmente todos los clientes serán visitados por el primer drone
    int DroneID[CustNum];
    for(i=0; i<CustNum; i++){
        DroneID[i] = 0;
    }

    // Creo un vector para ordenar las demandas
    vector<float> DemandasOrdenadas = Demand;
    // Borro las demandas del depósito
    DemandasOrdenadas.erase(DemandasOrdenadas.begin());
    DemandasOrdenadas.erase(DemandasOrdenadas.begin()+CustNum);
    // Ordeno las demandas de los clientes
    sort(DemandasOrdenadas.begin(), DemandasOrdenadas.end());

    // Verifico si la cantidad de clientes es par o impar
    //Par
    if(CustNum % 2 == 0){
        Q3 = (float)(3 * CustNum) / 4;
    }
    // Impar
    else{
        Q3 = (float)(3 * (CustNum + 1)) / 4;
    }

    // Defino la separación de demandas grandes a partir del tercer cuartil hacia arriba
    LimiteDemanda = DemandasOrdenadas[Q3-1];

    for(i=0; i<Largo; i++){
        if(Demand[i] >= LimiteDemanda){
            NodosPesados.push_back(NodeID[i]);
        }
    }

    LargoNodosPesados = NodosPesados.size();

    int ClientesPesados[LargoNodosPesados];
    for(i=0; i<LargoNodosPesados; i++){
        ClientesPesados[i] = NodosPesados[i];
    }

    while(true){
        // Si visito a todos los clientes, vuelvo al depósito y se termina la ejecución del algoritmo
        if(TodosVisitados(DroneID, CustNum)){
            Distancia = distancia(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0]);
            TiempoAcumulado += Distancia;
            EnergiaAcumulada += ConsumoEnergetico(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0], PesoCarga);
            if(EnergiaAcumulada > CapacidadEnergetica)
                EnergiaAcumulada = 0.268651;
            Ruta.push_back(NodeID[CustNum+1]);
            Tiempos_Nodos.push_back(TiempoAcumulado);
            Tiempos_Espera.push_back(0);
            Energia_Nodos.push_back(EnergiaAcumulada);
            Pesos_Nodos.push_back(0);
           
            // Si hay drones disponibles
            if(DronesUsados < DroneNum){
                DronesUsados++;
                Rutas.push_back(Ruta);
                Tiempos->push_back(Tiempos_Nodos);
                TiemposEspera->push_back(Tiempos_Espera);
                Energias->push_back(Energia_Nodos);
                Pesos->push_back(Pesos_Nodos);
            }

            *NumRutas = DronesUsados;
            break;
        }

        else{
            // Cambio de drone ya que DroneActual no se encuentra dentro de DroneID
            if(VerificarDrone(DroneID, CustNum, DroneActual)){

                // Inserto el nodo depósito final para diferenciar el drone
                if(NodoActual == NodeID[0]){
                    Ruta.pop_back();
                    Ruta.push_back(NodeID[CustNum+1]);
                }
                else{
                    Distancia = distancia(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0]);
                    TiempoAcumulado += Distancia;
                    EnergiaAcumulada += ConsumoEnergetico(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0], PesoCarga);
                    Ruta.push_back(NodeID[CustNum+1]);
                    Tiempos_Nodos.push_back(TiempoAcumulado);
                    Tiempos_Espera.push_back(0);
                    Energia_Nodos.push_back(EnergiaAcumulada);
                    Pesos_Nodos.push_back(0);
                   
                }

                // Verifico si tengo drones disponibles
                if(DroneActual < DroneNum)
                    DroneActual++;
                else{
                    break;
                }

                Rutas.push_back(Ruta);
                Tiempos->push_back(Tiempos_Nodos);
                TiemposEspera->push_back(Tiempos_Espera);
                Energias->push_back(Energia_Nodos);
                Pesos->push_back(Pesos_Nodos);

                // Reinicio la Ruta para el siguiente drone
                Ruta.clear();
                Tiempos_Nodos.clear();
                Tiempos_Espera.clear();
                Energia_Nodos.clear();
                Pesos_Nodos.clear();
                NoVisitados.clear();
                CapacidadAcumulada = 0;
                EnergiaAcumulada = 0;
                TiempoAcumulado = 0;
                PesoCarga = 1.5;
                PesoActual = 1.5;
                NodoActual = NodeID[0];
                NodoAnterior = NodoActual;
                Ruta.push_back(NodoActual);
                Tiempos_Nodos.push_back(TiempoAcumulado);
                Tiempos_Espera.push_back(0);
                Energia_Nodos.push_back(EnergiaAcumulada);
                Pesos_Nodos.push_back(PesoActual);
                if(DronesUsados < DroneNum)
                    DronesUsados++;
                else{
                    break;
                }
            }

            MasTemprano=999999;

            // Si visito a todos los clientes con demandas altas
            if(TodosVisitados(ClientesPesados, LargoNodosPesados) && Fase == 1){
                Distancia = distancia(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0]);
                TiempoAcumulado += Distancia;
                EnergiaAcumulada += ConsumoEnergetico(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0], PesoCarga);
                Energia_Nodos.push_back(EnergiaAcumulada);
                Tiempos_Espera.push_back(0);
                Tiempos_Nodos.push_back(TiempoAcumulado);
                Pesos_Nodos.push_back(0);
                PesoCarga = 1.5;
                PesoActual = 1.5;
                TiempoAcumulado = 0;
                EnergiaAcumulada = 0;
                Ruta.push_back(DepositoFinal);
                Fase++;
            }

            // Si estoy en la Fase 1 (la de crear una ruta para los clientes con demandas altas)
            if(Fase == 1){
                for(j=0; j<LargoNodosPesados; j++){
                    // Esta distancia solo se utiliza para printear
                    Distancia = distancia(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[j], Y_coor[j]);
                    if(ReadyTime[NodosPesados[j]] < MasTemprano && !existeEnVector(Visitados, NodosPesados[j]) && !existeEnVector(NoVisitados, NodosPesados[j])){
                        MasTemprano = ReadyTime[NodosPesados[j]];
                        NodoActual = NodosPesados[j];
                        PosicionPesada = j;
                    }
                }
            }

            // Si estoy en la Fase 2 (intentar insertar nodos en la ruta de las demandas pesadas)
            else if(Fase == 2){
                CapacidadAcumulada = Demand[Ruta[1]];
                EnergiaActual = Energia_Nodos[0];
                i=1;
                while(true){
                    if(Ruta[i] == DepositoFinal)
                        break;
                   
                    if(Ruta[i] == Deposito){
                        CapacidadAcumulada = 0;
                        EnergiaActual = 0;
                    }

                    else{
                        CapacidadAcumulada = Demand[Ruta[i]];
                        TiempoActual = Tiempos_Nodos[i-1];
                        for(j=1;j<Largo;j++){
                            if(!existeEnVector(Insertados, j)){
                                LimiteInferior = ReadyTime[NodeID[j]];
                                LimiteSuperior = DueTime[NodeID[j]];
                                // Restricción de la capacidad del drone
                                if((CapacidadAcumulada + Demand[j]) <= Capacidad){
                                    CapacidadAcumulada += Demand[j];
                                    Distancia = distancia(X_coor[Ruta[i-1]], Y_coor[Ruta[i-1]], X_coor[j], Y_coor[j]);
                                    Energia = ConsumoEnergetico(X_coor[Ruta[i-1]], Y_coor[Ruta[i-1]], X_coor[j], Y_coor[j], PesoCarga - Demand[j]);
                                    TiempoActual += Distancia;
                                    EnergiaActual += Energia;
                                    // Restricción de la ventana de tiempo y de la capacidad energética
                                    if(TiempoActual >= LimiteInferior && TiempoActual <= LimiteSuperior && EnergiaActual <= CapacidadEnergetica){
                                        // Se inserta al cliente y se actualizan los tiempos y energías
                                        Ruta.insert(Ruta.begin()+i, j);
                                        Tiempos_Nodos.insert(Tiempos_Nodos.begin()+i, TiempoActual);
                                        Tiempos_Espera.insert(Tiempos_Espera.begin()+i, 0);
                                        Energia_Nodos.insert(Energia_Nodos.begin()+i, EnergiaActual);
                                        Pesos_Nodos.insert(Pesos_Nodos.begin()+i, Demand[j]);
                                        RestriccionTiempo = NuevosTiempos(Ruta, &Tiempos_Nodos, &Energia_Nodos, NodeID, X_coor, Y_coor, ReadyTime, DueTime, i, CapacidadEnergetica, 0, 0, PesoCarga - Demand[j]);
                                        // Si es factible
                                        if(RestriccionTiempo){
                                            // Insertar el nodo
                                            Insertados.push_back(j);
                                            Visitados.push_back(j);
                                            DroneID[j-1] = -1;
                                            PesoCarga -= Demand[j];
                                            break;
                                        }
                                        // No es factible
                                        else{
                                            Ruta.erase(Ruta.begin()+i);
                                            Tiempos_Nodos.erase(Tiempos_Nodos.begin()+i);
                                            Tiempos_Espera.erase(Tiempos_Espera.begin()+i);
                                            Energia_Nodos.erase(Energia_Nodos.begin()+i);
                                            Pesos_Nodos.erase(Pesos_Nodos.begin()+i);
                                        }
                                    }

                                }
                               
                            }                            
                        }
                    }

                    i++;
                }

                Rutas.push_back(Ruta);
                Tiempos->push_back(Tiempos_Nodos);
                TiemposEspera->push_back(Tiempos_Espera);
                Energias->push_back(Energia_Nodos);
                Pesos->push_back(Pesos_Nodos);

                // Reinicio la Ruta para el siguiente drone
                Ruta.clear();
                Tiempos_Nodos.clear();
                Tiempos_Espera.clear();
                Energia_Nodos.clear();
                Pesos_Nodos.clear();
                NoVisitados.clear();
                CapacidadAcumulada = 0;
                EnergiaAcumulada += ConsumoEnergetico(X_coor[NodoActual], Y_coor[NodoActual], X_coor[0], Y_coor[0], PesoCarga);
                EnergiaAcumulada = 0;
                TiempoAcumulado = 0;
                PesoCarga = 1.5;
                PesoActual = 1.5;
                NodoActual = NodeID[0];
                NodoAnterior = NodoActual;
                Ruta.push_back(NodoActual);
                Tiempos_Nodos.push_back(TiempoAcumulado);
                Energia_Nodos.push_back(EnergiaAcumulada);
                Pesos_Nodos.push_back(PesoActual);
                if(DronesUsados < DroneNum)
                    DronesUsados++;
                else{
                    break;
                }
                Fase++;
               
            }

        }

        if(Fase == 3){
            for(j=1; j<Largo; j++){
                // Esta distancia solo se utiliza para printear
                Distancia = distancia(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[j], Y_coor[j]);
                if(ReadyTime[NodeID[j]] < MasTemprano && !existeEnVector(Visitados, NodeID[j]) && !existeEnVector(NoVisitados, NodeID[j])){
                    MasTemprano = ReadyTime[NodeID[j]];
                    NodoActual = NodeID[j];
                }
            }
        }

        // Sumo el peso del paquete del cliente correspondiente
        if(NodoActual != Deposito && NodoActual != DepositoFinal)
            CapacidadAcumulada += Demand[NodoActual];

        // Sumo la energía acumulada
        Energia = ConsumoEnergetico(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[NodoActual], Y_coor[NodoActual], PesoCarga - Demand[NodoActual]);
        EnergiaAcumulada += Energia;
   
        // El drone se queda sin paquetes o sin energía y debe volver al depósito para recargar la capacidad
        if(CapacidadAcumulada > Capacidad || EnergiaAcumulada > CapacidadEnergetica){
            EnergiaAcumulada -= Energia;
            EnergiaAcumulada += ConsumoEnergetico(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[0], Y_coor[0], PesoCarga);
            if(EnergiaAcumulada > CapacidadEnergetica)
                EnergiaDeposito = 0.264732;
            else
                EnergiaDeposito = EnergiaAcumulada;
            HaciaDeposito = true;
            EnergiaAcumulada = 0;
            CapacidadAcumulada = 0;
            PesoActual = 1.5;
            NodoActual = NodeID[0];
        }

        // Se calcula la distancia al próximo nodo y se visita
        Distancia = distancia(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[NodoActual], Y_coor[NodoActual]);
        TiempoAcumulado += Distancia;

        // Excedo el límite superior de la ventana de tiempo
        if(TiempoAcumulado > DueTime[NodoActual]){
            if(Fase == 1 && NodoActual != Deposito && NodoActual != DepositoFinal)
                ClientesPesados[PosicionPesada] = -1;
            TiempoAcumulado -= Distancia;
            CapacidadAcumulada -= Demand[NodoActual];
            EnergiaAcumulada -= ConsumoEnergetico(X_coor[NodoAnterior], Y_coor[NodoAnterior], X_coor[NodoActual], Y_coor[NodoActual], PesoCarga - Demand[NodoActual]);
            NoVisitados.push_back(NodoActual);
            // Queda pendiente para el siguiente drone
            DroneID[NodoActual-1] = DroneID[NodoActual-1] + 1;
            NodoActual = NodoAnterior;
        }

        else{
            Ruta.push_back(NodoActual);
            Visitados.push_back(NodoActual);
            // Se encuentra dentro del rango de la ventana de tiempo
            if(TiempoAcumulado >= ReadyTime[NodoActual] && TiempoAcumulado <= DueTime[NodoActual]){
                Tiempos_Espera.push_back(0);
                NodoAnterior = NodoActual;
                if(NodoActual != 0){
                    // Lo marco como visitado
                    DroneID[NodoActual-1] = -1;
                }
                if(Fase == 1 && NodoActual != Deposito && NodoActual != DepositoFinal)
                    ClientesPesados[PosicionPesada] = -1;
            }
            // No se encuentra dentro del rango de la ventana de tiempo
            else{
                Diferencia = ReadyTime[NodoActual] - TiempoAcumulado;
                Tiempos_Espera.push_back(Diferencia);
                // Si la nueva distancia está dentro del rango
                //if(Diferencia <= Holgura && Diferencia > 0){
                TiempoAcumulado = ReadyTime[NodoActual];
                NodoAnterior = NodoActual;
                // Lo marco como visitado
                DroneID[NodoActual-1] = -1;
                if(Fase == 1 && NodoActual != Deposito && NodoActual != DepositoFinal)
                    ClientesPesados[PosicionPesada] = -1;
                //}

                // Si la nueva distancia no está dentro del rango voy al depósito
               
            }

            Tiempos_Nodos.push_back(TiempoAcumulado);

            // Si voy en dirección al depósito
            if(HaciaDeposito){
                Energia_Nodos.push_back(EnergiaDeposito);
                PesoCarga = 1.5;
                PesoActual = 1.5;
                Pesos_Nodos.push_back(PesoActual);
                HaciaDeposito = false;
            }
            else{
                Energia_Nodos.push_back(EnergiaAcumulada);
                PesoCarga -= Demand[NodoActual];
                PesoActual -= Demand[NodoActual];
                if(PesoActual < 0.1)
                    PesoActual = 0;
                Pesos_Nodos.push_back(PesoActual);
            }

        }


    }

    return Rutas;

}

vector<vector<int>> HillClimbing(vector<vector<int>> Rutas, vector<vector<float>> *Tiempos, vector<vector<float>> TiemposEspera,
vector<vector<float>> *Energias, vector<vector<float>> *Pesos, vector<string> *Movimientos, vector<int> *Estadisticas, vector<int> *Iteraciones, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime,
vector<float> Demand, float Capacidad, float CapacidadEnergetica, float *FuncionEvaluacion, int NumRutas, int NumDrones, int delta, int tipo, float q){

    int UsoSwap = 0, UsoMoverCliente = 0, UsoMoverCliente2 = 0, UsoCrearRuta = 0, UsoCrearRuta2 = 0, UsoBalanceoCargaA1 = 0, UsoBalanceoCargaA2 = 0, UsoBalanceoCargaB = 0, cont = 1, NumRutasActual;
    int TotalSwap = 0, TotalMoverCliente = 0, TotalMoverCliente2 = 0, TotalCrearRuta = 0, TotalCrearRuta2 = 0, TotalBalanceoCargaA1 = 0, TotalBalanceoCargaA2 = 0, TotalBalanceoCargaB = 0;
    int RechazoBalanceoCargaA1 = 0, RechazoBalanceoCargaA2 = 0, RechazoBalanceoCargaB = 0, RechazoRestSwap = 0, RechazoCaliSwap = 0, RechazoRestMoverCliente = 0, RechazoCaliMoverCliente = 0;
    int RechazoRestMoverCliente2 = 0, RechazoCaliMoverCliente2 = 0, RechazoRestCrearRuta = 0, RechazoCaliCrearRuta = 0, RechazoRestCrearRuta2 = 0, RechazoCaliCrearRuta2 = 0;
    int Movimiento, DenominadorExploracion = 3, DenominadorExplotacion = 5;
    float NewFuncionEvaluacion = 999999, MejorFuncionEvaluacion = 999999, Random;
    bool Factible = false;

    vector<vector<int>> RutasOperador, RutaOriginal = Rutas, RutasAnteriores, MejorRutas;
    vector<vector<float>> NewTiempos = *Tiempos;
    vector<vector<float>> NewEnergias = *Energias;
    vector<vector<float>> NewPesos = *Pesos;
    vector<vector<float>> TiemposActuales, EnergiasActuales, PesosActuales, MejorTiempos, MejorEnergias, MejorPesos;
    vector<int> NodosSwap, RutaMovida, NumeradoresExploracion, NumeradoresExplotacion;
    vector<float> ProbAcumuladasExploracion, ProbAcumuladasExplotacion;

    // Inicialización probabilidades acumuladas de los movimientos

    // Fase exploración
    ProbAcumuladasExploracion.push_back(0.33);
    ProbAcumuladasExploracion.push_back(0.66);
    ProbAcumuladasExploracion.push_back(1.0);

    NumeradoresExploracion.push_back(1);
    NumeradoresExploracion.push_back(1);
    NumeradoresExploracion.push_back(1);

    // Fase explotación
    ProbAcumuladasExplotacion.push_back(0.2);
    ProbAcumuladasExplotacion.push_back(0.4);
    ProbAcumuladasExplotacion.push_back(0.6);
    ProbAcumuladasExplotacion.push_back(0.8);
    ProbAcumuladasExplotacion.push_back(1.0);

    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);

    time_t start, end, timeTaken;
    int TiempoEjecucion = 30; //300; // 5 minutos
    int timeLeft = TiempoEjecucion, timeLeft2 = TiempoEjecucion;
    int IteracionesTotales = 3, Iteracion = 1;

    while(Iteracion <= IteracionesTotales){

        // Fase 1: Fase de exploración
        start = time(0);
        timeLeft = TiempoEjecucion;

        //while(true){
        while(timeLeft > 0){

            end = time(0);
            timeTaken = end-start;
            timeLeft = TiempoEjecucion - timeTaken;

            // Se selecciona un número flotante al azar entre 0 y 1
            Random = float_rand(0,1);
            Movimiento = RuletaMovimientos(ProbAcumuladasExploracion, Random);

            // Movimiento: BalanceoCargaA1
            if(Random >= 0 && Random < 0.33){
                TotalBalanceoCargaA1++;
                RutasOperador = BalanceoCargaA1(Rutas, NumRutas, &NodosSwap, &RutaMovida);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                if(NodosSwap[2] != -1){
                    NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                    Factible = RestriccionesBalanceoCargaA1(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, RutaMovida, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);
                }

                else{
                    Factible = false;
                }

                if(Factible){
                    UsoBalanceoCargaA1++;
                    Movimientos->push_back("BalanceoCargaA1");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                    Rutas = RutasOperador;
                }
                else{
                    RechazoBalanceoCargaA1++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: BalanceoCargaA2
            else if(Random >= 0.33 && Random < 0.66){
                TotalBalanceoCargaA2++;
                RutasOperador = BalanceoCargaA2(Rutas, NumRutas, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesBalanceoCargaA2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(Factible){
                    UsoBalanceoCargaA2++;
                    Movimientos->push_back("BalanceoCargaA2");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                    Rutas = RutasOperador;
                }

                else{
                    RechazoBalanceoCargaA2++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: BalanceoCargaB
            else if(Random >= 0.66 && Random <= 1){
                TotalBalanceoCargaB++;
                RutasOperador = BalanceoCargaB(Rutas, NumRutas, NumDrones, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesBalanceoCargaB(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(Factible){
                    UsoBalanceoCargaB++;
                    Movimientos->push_back("BalanceoCargaB");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    ProbAcumuladasExploracion = ActualizarProbabilidades(&NumeradoresExploracion, &DenominadorExploracion, Movimiento);
                    Rutas = RutasOperador;
                }

                else{
                    RechazoBalanceoCargaB++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            cont++;

        }

        // Fase 2: Fase de explotación
        start = time(0);
        timeLeft2 = TiempoEjecucion;

        //while(true){
        while(timeLeft2 > 0){

            end = time(0);
            timeTaken = end-start;
            timeLeft2 = TiempoEjecucion - timeTaken;

            // Se selecciona un número flotante al azar entre 0 y 1
            Random = float_rand(0,1);
            Movimiento = RuletaMovimientos(ProbAcumuladasExplotacion, Random);

            // Movimiento: Swap
            if(Movimiento == 0){
                TotalSwap++;
                RutasOperador = Swap(Rutas, NumRutas, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesSwap(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    UsoSwap++;
                    Movimientos->push_back("Swap");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);

                    if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                        MejorRutas = Rutas;
                        MejorFuncionEvaluacion = *FuncionEvaluacion;
                        MejorTiempos = NewTiempos;
                        MejorEnergias = NewEnergias;
                        MejorPesos = NewPesos;
                    }
                }
                else{
                    if(Factible == false)
                        RechazoRestSwap++;
                    if(NewFuncionEvaluacion >= *FuncionEvaluacion)
                        RechazoCaliSwap++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: MoverCliente
            else if(Movimiento == 1){
                TotalMoverCliente++;
                RutasOperador = MoverCliente(Rutas, NumRutas, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesMover(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    UsoMoverCliente++;
                    Movimientos->push_back("MoverCliente");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);

                    if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                        MejorRutas = Rutas;
                        MejorFuncionEvaluacion = *FuncionEvaluacion;
                        MejorTiempos = NewTiempos;
                        MejorEnergias = NewEnergias;
                        MejorPesos = NewPesos;
                    }
                }
                else{
                    if(Factible == false)
                        RechazoRestMoverCliente++;
                    if(NewFuncionEvaluacion >= *FuncionEvaluacion)
                        RechazoCaliMoverCliente++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: MoverCliente2
            else if(Movimiento == 2){
                TotalMoverCliente2++;
                RutasOperador = MoverCliente2(Rutas, NumRutas, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesMover2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(NewFuncionEvaluacion <= *FuncionEvaluacion && Factible){
                    UsoMoverCliente2++;
                    Movimientos->push_back("MoverCliente2");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);

                    if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                        MejorRutas = Rutas;
                        MejorFuncionEvaluacion = *FuncionEvaluacion;
                        MejorTiempos = NewTiempos;
                        MejorEnergias = NewEnergias;
                        MejorPesos = NewPesos;
                    }
                }
                else{
                    if(Factible == false)
                        RechazoRestMoverCliente2++;
                    if(NewFuncionEvaluacion > *FuncionEvaluacion)
                        RechazoCaliMoverCliente2++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: CrearRuta
            else if(Movimiento == 3){
                TotalCrearRuta++;
                RutasOperador = CrearRuta(Rutas, NumRutas, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesCrear(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    UsoCrearRuta++;
                    Movimientos->push_back("CrearRuta");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);

                    if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                        MejorRutas = Rutas;
                        MejorFuncionEvaluacion = *FuncionEvaluacion;
                        MejorTiempos = NewTiempos;
                        MejorEnergias = NewEnergias;
                        MejorPesos = NewPesos;
                    }
                }
                else{
                    if(Factible == false)
                        RechazoRestCrearRuta++;
                    if(NewFuncionEvaluacion >= *FuncionEvaluacion)
                        RechazoCaliCrearRuta++;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: CrearRuta2
            else if(Movimiento == 4){
                TotalCrearRuta2++;
                NumRutasActual = NumRutas;
                RutasOperador = CrearRuta2(Rutas, &NumRutas, NumDrones, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas, X_coor, Y_coor, delta, tipo, q);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesCrear2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                if(NewFuncionEvaluacion <= *FuncionEvaluacion && Factible){
                    UsoCrearRuta2++;
                    Movimientos->push_back("CrearRuta2");
                    Iteraciones->push_back(cont);
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);

                    if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                        MejorRutas = Rutas;
                        MejorFuncionEvaluacion = *FuncionEvaluacion;
                        MejorTiempos = NewTiempos;
                        MejorEnergias = NewEnergias;
                        MejorPesos = NewPesos;
                    }
                }
                else{
                    if(Factible == false)
                        RechazoRestCrearRuta2++;
                    if(NewFuncionEvaluacion > *FuncionEvaluacion)
                        RechazoCaliCrearRuta2++;
                    NumRutas = NumRutasActual;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            cont++;

        }

        Iteracion++;
    }

    // Guardar las estadísticas
    Estadisticas->push_back(UsoSwap);
    Estadisticas->push_back(TotalSwap);
    Estadisticas->push_back(UsoMoverCliente);
    Estadisticas->push_back(TotalMoverCliente);
    Estadisticas->push_back(UsoMoverCliente2);
    Estadisticas->push_back(TotalMoverCliente2);
    Estadisticas->push_back(UsoCrearRuta);
    Estadisticas->push_back(TotalCrearRuta);
    Estadisticas->push_back(UsoCrearRuta2);
    Estadisticas->push_back(TotalCrearRuta2);
    Estadisticas->push_back(UsoBalanceoCargaA1);
    Estadisticas->push_back(TotalBalanceoCargaA1);
    Estadisticas->push_back(UsoBalanceoCargaA2);
    Estadisticas->push_back(TotalBalanceoCargaA2);
    Estadisticas->push_back(UsoBalanceoCargaB);
    Estadisticas->push_back(TotalBalanceoCargaB);
    Estadisticas->push_back(RechazoBalanceoCargaA1);
    Estadisticas->push_back(RechazoBalanceoCargaA2);
    Estadisticas->push_back(RechazoBalanceoCargaB);
    Estadisticas->push_back(RechazoRestSwap);
    Estadisticas->push_back(RechazoCaliSwap);
    Estadisticas->push_back(RechazoRestMoverCliente);
    Estadisticas->push_back(RechazoCaliMoverCliente);
    Estadisticas->push_back(RechazoRestMoverCliente2);
    Estadisticas->push_back(RechazoCaliMoverCliente2);
    Estadisticas->push_back(RechazoRestCrearRuta);
    Estadisticas->push_back(RechazoCaliCrearRuta);
    Estadisticas->push_back(RechazoRestCrearRuta2);
    Estadisticas->push_back(RechazoCaliCrearRuta2);
    Estadisticas->push_back(cont-1);

    Rutas = MejorRutas;
    *FuncionEvaluacion = MejorFuncionEvaluacion;
    *Tiempos = MejorTiempos;
    *Energias = MejorEnergias;
    *Pesos = MejorPesos;

    return Rutas;
}

vector<vector<int>> SimulatedAnnealing(vector<vector<int>> Rutas, vector<vector<float>> *Tiempos,
vector<vector<float>> *Energias, vector<vector<float>> *Pesos, int *IteracionesTotales, vector<int> *EstadisticasSA, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime,
vector<float> Demand, float Capacidad, float CapacidadEnergetica, float *FuncionEvaluacion, int *NumRutas, int NumDrones, int delta, int tipo, float q, vector<float> *Temperaturas, vector<float> *Calidades, vector<int> *Evaluaciones,
int Iteraciones_SA, int Iteraciones_CambioT, float Temperatura_Inicial, float Factor_Enfriamiento){

    int NumRutas_SA = *NumRutas, NumRutasActual, Movimiento, DenominadorExplotacion = 5, ContadorIteraciones = 0, ContadorCambioTemp = 0, ContadorFaseExploracion = 0, IterTotal = *IteracionesTotales;
    float NewFuncionEvaluacion = 999999, Random, ProbabilidadAceptacion, MejorFuncionEvaluacion = *FuncionEvaluacion;
    bool Factible = false;

    vector<vector<int>> RutasOperador, MejorRutas = Rutas;
    vector<vector<float>> NewTiempos = *Tiempos;
    vector<vector<float>> NewEnergias = *Energias;
    vector<vector<float>> NewPesos = *Pesos;
    vector<vector<float>> TiemposActuales, EnergiasActuales, PesosActuales, MejorTiempos = NewTiempos, MejorEnergias = NewEnergias, MejorPesos = NewPesos;;
    vector<int> NodosSwap, RutaMovida, NumeradoresExploracion, NumeradoresExplotacion, Estadisticas_SA = *EstadisticasSA, Evaluaciones_SA = *Evaluaciones;
    vector<float> ProbAcumuladasExploracion, ProbAcumuladasExplotacion, Temperaturas_SA = *Temperaturas, Calidades_SA = *Calidades;
   
    // Inicialización probabilidades acumuladas de los movimientos

    // Fase explotación
    ProbAcumuladasExplotacion.push_back(0.2);
    ProbAcumuladasExplotacion.push_back(0.4);
    ProbAcumuladasExplotacion.push_back(0.6);
    ProbAcumuladasExplotacion.push_back(0.8);
    ProbAcumuladasExplotacion.push_back(1.0);

    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);
    NumeradoresExplotacion.push_back(1);

    // Parámetros Simulated Annealing
    float T_actual = Temperatura_Inicial;

    while(ContadorIteraciones < Iteraciones_SA){
        while(ContadorCambioTemp < Iteraciones_CambioT){

            IterTotal++;

            // Se selecciona un número flotante al azar entre 0 y 1
            Random = float_rand(0,1);
            Movimiento = RuletaMovimientos(ProbAcumuladasExplotacion, Random);

            // Movimiento: Swap
            if(Movimiento == 0){
                Estadisticas_SA[1]++;
                RutasOperador = Swap(Rutas, NumRutas_SA, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas_SA, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_SA.push_back(T_actual);
                Calidades_SA.push_back(NewFuncionEvaluacion);
                Evaluaciones_SA.push_back(IterTotal);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesSwap(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                // Mejora la calidad de la solución y es factible
                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    Estadisticas_SA[0]++;
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                }

                // Empeora la calidad de la solución y es factible
                else if(NewFuncionEvaluacion >= *FuncionEvaluacion && Factible){
                    Random = float_rand(0,1);
                    ProbabilidadAceptacion = exp((*FuncionEvaluacion - NewFuncionEvaluacion) / T_actual);
                    if(Random < ProbabilidadAceptacion){
                        Estadisticas_SA[0]++;
                        *FuncionEvaluacion = NewFuncionEvaluacion;
                        Rutas = RutasOperador;
                        ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                    }
                    else{
                        NewTiempos = TiemposActuales;
                        NewEnergias = EnergiasActuales;
                        NewPesos = PesosActuales;
                    }
                }

                // No es factible
                else{
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }

            }

            // Movimiento: MoverCliente
            else if(Movimiento == 1){
                Estadisticas_SA[3]++;
                RutasOperador = MoverCliente(Rutas, NumRutas_SA, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas_SA, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_SA.push_back(T_actual);
                Calidades_SA.push_back(NewFuncionEvaluacion);
                Evaluaciones_SA.push_back(IterTotal);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesMover(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                // Mejora la calidad de la solución y es factible
                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    Estadisticas_SA[2]++;
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                }

                // Empeora la calidad de la solución y es factible
                else if(NewFuncionEvaluacion >= *FuncionEvaluacion && Factible){
                    Random = float_rand(0,1);
                    ProbabilidadAceptacion = exp((*FuncionEvaluacion - NewFuncionEvaluacion) / T_actual);
                    if(Random < ProbabilidadAceptacion){
                        Estadisticas_SA[2]++;
                        *FuncionEvaluacion = NewFuncionEvaluacion;
                        Rutas = RutasOperador;
                        ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                    }
                    else{
                        NewTiempos = TiemposActuales;
                        NewEnergias = EnergiasActuales;
                        NewPesos = PesosActuales;
                    }
                }

                // No es factible
                else{
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: MoverCliente2
            else if(Movimiento == 2){
                Estadisticas_SA[5]++;
                RutasOperador = MoverCliente2(Rutas, NumRutas_SA, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas_SA, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_SA.push_back(T_actual);
                Calidades_SA.push_back(NewFuncionEvaluacion);
                Evaluaciones_SA.push_back(IterTotal);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesMover2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                // Mejora la calidad de la solución y es factible
                if(NewFuncionEvaluacion <= *FuncionEvaluacion && Factible){
                    Estadisticas_SA[4]++;
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                }

                // Empeora la calidad de la solución y es factible
                else if(NewFuncionEvaluacion > *FuncionEvaluacion && Factible){
                    Random = float_rand(0,1);
                    ProbabilidadAceptacion = exp((*FuncionEvaluacion - NewFuncionEvaluacion) / T_actual);
                    if(Random < ProbabilidadAceptacion){
                        Estadisticas_SA[4]++;
                        *FuncionEvaluacion = NewFuncionEvaluacion;
                        Rutas = RutasOperador;
                        ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                    }
                    else{
                        NewTiempos = TiemposActuales;
                        NewEnergias = EnergiasActuales;
                        NewPesos = PesosActuales;
                    }
                }

                // No es factible
                else{
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: CrearRuta
            else if(Movimiento == 3){
                Estadisticas_SA[7]++;
                RutasOperador = CrearRuta(Rutas, NumRutas_SA, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas_SA, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_SA.push_back(T_actual);
                Calidades_SA.push_back(NewFuncionEvaluacion);
                Evaluaciones_SA.push_back(IterTotal);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesCrear(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                // Mejora la calidad de la solución y es factible
                if(NewFuncionEvaluacion < *FuncionEvaluacion && Factible){
                    Estadisticas_SA[6]++;
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                }

                // Empeora la calidad de la solución y es factible
                else if(NewFuncionEvaluacion >= *FuncionEvaluacion && Factible){
                    Random = float_rand(0,1);
                    ProbabilidadAceptacion = exp((*FuncionEvaluacion - NewFuncionEvaluacion) / T_actual);
                    if(Random < ProbabilidadAceptacion){
                        Estadisticas_SA[6]++;
                        *FuncionEvaluacion = NewFuncionEvaluacion;
                        Rutas = RutasOperador;
                        ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                    }
                    else{
                        NewTiempos = TiemposActuales;
                        NewEnergias = EnergiasActuales;
                        NewPesos = PesosActuales;
                    }
                }

                // No es factible
                else{
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Movimiento: CrearRuta2
            else if(Movimiento == 4){
                Estadisticas_SA[9]++;
                NumRutasActual = NumRutas_SA;
                RutasOperador = CrearRuta2(Rutas, &NumRutas_SA, NumDrones, &NodosSwap);
                NewFuncionEvaluacion = FuncionEvaluacionTotal(RutasOperador, NumRutas_SA, X_coor, Y_coor, delta, tipo, q);
                Temperaturas_SA.push_back(T_actual);
                Calidades_SA.push_back(NewFuncionEvaluacion);
                Evaluaciones_SA.push_back(IterTotal);
                TiemposActuales = NewTiempos;
                EnergiasActuales = NewEnergias;
                PesosActuales = NewPesos;
                Factible = RestriccionesCrear2(RutasOperador, NewTiempos, NewEnergias, NewPesos, NodosSwap, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, delta, tipo);

                // Mejora la calidad de la solución y es factible
                if(NewFuncionEvaluacion <= *FuncionEvaluacion && Factible){
                    Estadisticas_SA[8]++;
                    *FuncionEvaluacion = NewFuncionEvaluacion;
                    Rutas = RutasOperador;
                    ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                }

                // Empeora la calidad de la solución y es factible
                else if(NewFuncionEvaluacion > *FuncionEvaluacion && Factible){
                    Random = float_rand(0,1);
                    ProbabilidadAceptacion = exp((*FuncionEvaluacion - NewFuncionEvaluacion) / T_actual);
                    if(Random < ProbabilidadAceptacion){
                        Estadisticas_SA[8]++;
                        *FuncionEvaluacion = NewFuncionEvaluacion;
                        Rutas = RutasOperador;
                        ProbAcumuladasExplotacion = ActualizarProbabilidades(&NumeradoresExplotacion, &DenominadorExplotacion, Movimiento);
                    }
                    else{
                        NumRutas_SA = NumRutasActual;
                        NewTiempos = TiemposActuales;
                        NewEnergias = EnergiasActuales;
                        NewPesos = PesosActuales;
                    }
                }

                // No es factible
                else{
                    NumRutas_SA = NumRutasActual;
                    NewTiempos = TiemposActuales;
                    NewEnergias = EnergiasActuales;
                    NewPesos = PesosActuales;
                }
            }

            // Si la solución actual mejora con respecto a la mejor hasta el momento
            if(*FuncionEvaluacion < MejorFuncionEvaluacion){
                MejorRutas = Rutas;
                MejorFuncionEvaluacion = *FuncionEvaluacion;
                MejorTiempos = NewTiempos;
                MejorEnergias = NewEnergias;
                MejorPesos = NewPesos;
                ContadorFaseExploracion = 0;
            }

            ContadorCambioTemp++;
            ContadorIteraciones++;
            ContadorFaseExploracion++;
        }

        T_actual = T_actual * Factor_Enfriamiento;
        ContadorCambioTemp = 0;

        // if(T_actual < 1)
        //     IteracionBajaTemp = ContadorIteraciones;

    }

    Rutas = MejorRutas;
    *FuncionEvaluacion = MejorFuncionEvaluacion;
    *Tiempos = MejorTiempos;
    *Energias = MejorEnergias;
    *Pesos = MejorPesos;
    *IteracionesTotales = IterTotal;
    *EstadisticasSA = Estadisticas_SA;
    *Temperaturas = Temperaturas_SA;
    *Calidades = Calidades_SA;
    *Evaluaciones = Evaluaciones_SA;
    *NumRutas = NumRutas_SA;

    return Rutas;

}

vector<vector<int>> IteratedLocalSearch(vector<vector<int>> Rutas, vector<vector<float>> *Tiempos,
vector<vector<float>> *Energias, vector<vector<float>> *Pesos, int *IteracionesTotales, vector<int> *EstadisticasPerturbacion, vector<int> *EstadisticasSA, vector<int> X_coor, vector<int> Y_coor, vector<int> ReadyTime, vector<int> DueTime,
vector<float> Demand, float Capacidad, float CapacidadEnergetica, float *FuncionEvaluacion, int NumRutas, int NumDrones, int delta, int tipo, float q, vector<float> *Temperaturas, vector<float> *Calidades, vector<int> *Evaluaciones,
int Iteraciones_ILS, int Iteraciones_SA, int Iteraciones_CambioT, int Movimientos_Perturbacion, float Temperatura_Inicial, float Factor_Enfiramiento){

    int Contador = 0, Iteraciones_Totales = *IteracionesTotales;
    float FuncionEvaluacion_act = *FuncionEvaluacion, MejorFuncionEvaluacion;
    vector<int> Estadisticas_SA = *EstadisticasSA, Estadisticas_Perturbacion = *EstadisticasPerturbacion, Evaluaciones_SA = *Evaluaciones;
    vector<float> Temperaturas_SA = *Temperaturas, Calidades_SA = *Calidades;
    vector<vector<int>> Solucion_Actual, Mejor_Solucion, Solucion_Perturbada, Solucion_SA;
    vector<vector<float>> Tiempos_act = *Tiempos, Energias_act = *Energias, Pesos_act = *Pesos, MejorTiempos, MejorEnergias, MejorPesos, TiemposRepaldo, EnergiasRespaldo, PesosRespaldo;

    Solucion_Actual = SimulatedAnnealing(Rutas, &Tiempos_act, &Energias_act, &Pesos_act, &Iteraciones_Totales, &Estadisticas_SA, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, &FuncionEvaluacion_act, &NumRutas, NumDrones, delta, tipo, 0, &Temperaturas_SA, &Calidades_SA, &Evaluaciones_SA, Iteraciones_SA, Iteraciones_CambioT, Temperatura_Inicial, Factor_Enfriamiento);
    Mejor_Solucion = Solucion_Actual;
    MejorFuncionEvaluacion = FuncionEvaluacion_act;

    MejorTiempos = Tiempos_act;
    MejorEnergias = Energias_act;
    MejorPesos = Pesos_act;

    while(Contador < Iteraciones_ILS){

        TiemposRepaldo = MejorTiempos;
        EnergiasRespaldo = MejorEnergias;
        PesosRespaldo = MejorPesos;
        NumRutas = Mejor_Solucion.size();
        Solucion_Perturbada = Perturbacion(Mejor_Solucion, &MejorTiempos, &MejorEnergias, &MejorPesos, &Iteraciones_Totales, &Estadisticas_Perturbacion, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, &FuncionEvaluacion_act, NumRutas, NumDrones, delta, tipo, 0, Movimientos_Perturbacion, &Temperaturas_SA, &Calidades_SA, &Evaluaciones_SA);
        Tiempos_act = MejorTiempos;
        Energias_act = MejorEnergias;
        Pesos_act = MejorPesos;
        Solucion_SA = SimulatedAnnealing(Solucion_Perturbada, &Tiempos_act, &Energias_act, &Pesos_act, &Iteraciones_Totales, &Estadisticas_SA, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, &FuncionEvaluacion_act, &NumRutas, NumDrones, delta, tipo, 0,  &Temperaturas_SA, &Calidades_SA, &Evaluaciones_SA, Iteraciones_SA, Iteraciones_CambioT, Temperatura_Inicial, Factor_Enfriamiento);

        if(FuncionEvaluacion_act < MejorFuncionEvaluacion){
            Mejor_Solucion = Solucion_SA;
            MejorFuncionEvaluacion = FuncionEvaluacion_act;
            MejorTiempos = Tiempos_act;
            MejorEnergias = Energias_act;
            MejorPesos = Pesos_act;
        }
        else{
            MejorTiempos = TiemposRepaldo;
            MejorEnergias = EnergiasRespaldo;
            MejorPesos = PesosRespaldo;
        }

        Contador++;
    }

    *FuncionEvaluacion = MejorFuncionEvaluacion;
    *Tiempos = MejorTiempos;
    *Energias = MejorEnergias;
    *Pesos = MejorPesos;
    *IteracionesTotales = Iteraciones_Totales;
    *EstadisticasPerturbacion = Estadisticas_Perturbacion;
    *EstadisticasSA = Estadisticas_SA;
    *Temperaturas = Temperaturas_SA;
    *Calidades = Calidades_SA;
    *Evaluaciones = Evaluaciones_SA;

    return Mejor_Solucion;

}

// ************************************************************************************************************ //
// *************************************************** MAIN *************************************************** //
// ************************************************************************************************************ //

int main(int argc, char **argv){

    // Guardar los parámetros introducidos
    Capture_Params(argc, argv);
    srand48(Seed);

    // Parámetros
    int Delta = 360; // Costo unitario de energía
    int Tipo = 0; // 0 => Tiempo total, 1 => Energía total, 2 => Tiempo + Energía total
    float Capacidad = 1.5; // Capacidad del drone
    float CapacidadEnergetica = 0.27; // Capacidad energética de la batería

    int CustNum, DroneNum, NumRutas, IteracionesTotales = 0, NumeroInstancia, PosicionEstadoDelArte;
    float FuncionEvaluacion, GapPorcentual, SolucionAutores;
    string NombreInstancia, InstanciaAux, token, delimiter = "_";
    vector<int> NodeID, X_coor, Y_coor, ReadyTime, DueTime, NodosSwap, EstadisticasPerturbacion(6, 0), EstadisticasSA(10, 0), Iteraciones, VisitadosDrones, Evaluaciones;
    vector<float> Demand, Tiempos_Nodos, Temperaturas, Calidades;
    vector<string> Movimientos;
    vector<vector<int>> Rutas, Rutas_SA, Rutas_ILS;
    vector<vector<float>> Tiempos, TiemposEspera, Energias, Pesos;
    size_t pos = 0;
    float Sol_EstadoDelArte[40] = {2930.4, 4426.1, 4252.3, 4105.3, 4225, 6601.6, 4114.7, 5575, 5125.9, 6900, 7720.1, 8912.6, 8219.7, 6229.5, 7269.4, 9962.5, 8064.6, 9422.6, 9460.6, 10398.1, 9165.8, 11811.4, 11530.4, 11670.9, 11330.9, 11719.2, 12145.5, 12346.4, 12925.3, 12384.3, 14546.6, 15741.8, 13230.2, 15189.5, 14554.9, 13832.5, 18423.5, 17548.7, 15231.5, 17467};

    NombreInstancia.append(Instancia);
    InstanciaAux = NombreInstancia;

    while ((pos = InstanciaAux.find(delimiter)) != string::npos){
        token = InstanciaAux.substr(0, pos);
        InstanciaAux.erase(0, pos + delimiter.length());
    }
    NumeroInstancia = stoi(InstanciaAux);

    // Se lee la instancia y se guardan los datos en sus respectivas variables
    leerInstancia(NombreInstancia, &CustNum, &DroneNum, &NodeID, &X_coor, &Y_coor, &Demand, &ReadyTime, &DueTime);

    if(CustNum == 10)
        PosicionEstadoDelArte = -1;
    else if(CustNum == 15)
        PosicionEstadoDelArte = 4;
    else if(CustNum == 20)
        PosicionEstadoDelArte = 9;
    else if(CustNum == 25)
        PosicionEstadoDelArte = 14;
    else if(CustNum == 30)
        PosicionEstadoDelArte = 19;
    else if(CustNum == 35)
        PosicionEstadoDelArte = 24;
    else if(CustNum == 40)
        PosicionEstadoDelArte = 29;
    else if(CustNum == 45)
        PosicionEstadoDelArte = 34;

    PosicionEstadoDelArte += NumeroInstancia;
    SolucionAutores = Sol_EstadoDelArte[PosicionEstadoDelArte];

    Rutas = Greedy(NodeID, X_coor, Y_coor, ReadyTime, DueTime, Demand, &Tiempos, &TiemposEspera, &Energias, &Pesos, CustNum, DroneNum, Capacidad, CapacidadEnergetica, &NumRutas);

    FuncionEvaluacion = DistanciaTotal(Rutas, X_coor, Y_coor);
    VisitadosDrones = ClientesDrones(Rutas);

    Rutas_ILS = IteratedLocalSearch(Rutas, &Tiempos, &Energias, &Pesos, &IteracionesTotales, &EstadisticasPerturbacion, &EstadisticasSA, X_coor, Y_coor, ReadyTime, DueTime, Demand, Capacidad, CapacidadEnergetica, &FuncionEvaluacion, NumRutas, DroneNum, Delta, Tipo, 0, &Temperaturas, &Calidades, &Evaluaciones, Iteraciones_ILS, Iteraciones_SA, Iteraciones_CambioT, Movimientos_Perturbacion, Temperatura_Inicial, Factor_Enfriamiento);

    GapPorcentual = 100*((abs(SolucionAutores - FuncionEvaluacion))/SolucionAutores);
    cout << fixed << setprecision(2) << GapPorcentual << endl;

    return 0;
}