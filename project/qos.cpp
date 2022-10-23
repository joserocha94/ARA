#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ctime>  
#include <chrono> 
#include <thread>
#include <fstream>


#define MAX_NODES 9999
#define TIME_CONST 1

#define EDGE_MAX_WIDTH 999
#define EDGE_MIN_WIDTH 0
#define EDGE_MAX_LENGTH 999
#define EDGE_MIN_LENGTH 0


struct Edge
{
    int source;
    int target;
    int width;
    int length;
};

struct Node
{
    int id;
    std::vector<Edge> in_edges;
    std::vector<Edge> out_edges;
};

struct Graph
{
    int n;
    std::vector<Node> nodes;
};

struct Event
{
    double time;
    Edge event;
};

struct Calendar
{
    std::vector<Event> list;
};

struct Info
{
    int width;
    int length;
    int it;
    double time;
};

struct candidate_pair
{
    int width;
    int length;
    int label;
    int next_label;
};

std::vector<std::vector<std::vector<candidate_pair>>> cd;
std::vector<std::vector<std::vector<int>>> ninja_parent;
std::vector<std::vector<Info>> dist;
std::vector<std::vector<int>> parent;

Graph network;
Calendar calendar;

int global_iteration;
bool debug, data, sw;


/*  função ccdf para análise dos tempos de descoberta
ficheiro produzido disponibilizado em root/output   */
void ccdf_times(std::string algoritmo)
{

    std::string filename = "output/ccdf_times_" + algoritmo + ".txt";
    std::ofstream outfile (filename);

    for (int i=0; i<network.n; i++)
        for (int j=0; j<network.n; j++)
            if (dist[i][j].length != EDGE_MAX_LENGTH && dist[i][j].width != EDGE_MAX_WIDTH)
                outfile << dist[i][j].time <<std::endl;

    outfile.close();
}


/*  função ccdf para nálise das larguras dos nós
encontrados pela execução do programa. o ficheiro 
produzido fica disponibilizado em root/output   */
void ccdf_width(std::string algoritmo)
{

    std::string filename = "output/ccdf_width_" + algoritmo + ".txt";
    std::ofstream outfile (filename);

    for (int i=0; i<network.n; i++)
        for (int j=0; j<network.n; j++)
            if (dist[i][j].length != EDGE_MAX_LENGTH && dist[i][j].width != EDGE_MAX_WIDTH)
                outfile << dist[i][j].width <<std::endl;

    outfile.close();
}


/*  função ccdf para nálise das distâncias dos nós
encontrados pela execução do programa. o ficheiro 
produzido fica disponibilizado em root/output   */
void ccdf_length(std::string algoritmo)
{

    std::string filename = "output/ccdf_length_" + algoritmo + ".txt";
    std::ofstream outfile (filename);

    for (int i=0; i<network.n; i++)
        for (int j=0; j<network.n; j++)
            if (dist[i][j].length != EDGE_MAX_LENGTH && dist[i][j].width != EDGE_MAX_WIDTH)
                outfile << dist[i][j].length <<std::endl;

    outfile.close();
}


/* função ccdf para execução de todas as funções ccdf*/
void ccdf(std::string algoritmo)
{
    ccdf_times(algoritmo);
    ccdf_width(algoritmo);
    ccdf_length(algoritmo);
}


/*  distribuição uniforme utilizada para cálculo
do atraso aleatório na criação de um novo evento e posterior
inserção no calendário  */
double get_random()
{
    return ((double)rand() / (RAND_MAX));
}


/*  função para imprimir o calendário utilizado pelo simulador. 
como o calendário estás sempre ordenado também o output estará  */
void print_calendar(Calendar calendar)
{
    printf("\n\n");
    for (int i = 0; i < calendar.list.size(); i++)
        std::cout << "\tevento " << i << ": ("
        << calendar.list[i].event.source << " > "
        << calendar.list[i].event.target << ") : ("
        << calendar.list[i].event.width << ","
        << calendar.list[i].event.length << ") : "
        << calendar.list[i].time
        << std::endl;
}


/*  imprime pares (largura, distância) de cada nó para todas
as outras sources. inclui nós não visitados com default values  */
void print_distances()
{
    for (int i = 0; i < dist.size(); i++)
    {
        printf("\n");
        for (int j = 0; j < network.n; j++)
            printf("\t(%4d; %4d) at %2d in %.2f", 
                dist[i][j].width, 
                dist[i][j].length, 
                dist[i][j].it, 
                dist[i][j].time);
    }
    printf("\n");
}


/*  imprime distância específica entre uma origem e 
um destino. utilizado no modo interativo */
void print_distances(int source, int destination)
{
    int width = dist[source][destination].width;
    int length = dist[source][destination].length;
    
    printf("was found with (%2d, %2d)", width, length);
}


/* destroi dados da matrix de distâncias. reconstroi
a matrix para posterior execução por um outro algoritmo */
void reset_distances()
{
    global_iteration = 0;
    dist.resize(0);
    dist.resize(network.n);

    Info dummy = { EDGE_MAX_WIDTH, EDGE_MAX_LENGTH, 0, 0 };

    for (int i = 0; i < dist.size(); i++)
        for (int j = 0; j < network.n; j++)
            dist[i].push_back(dummy);
}


/* destroi dados da matrix dos predecessores. reconstroi
a matrix para posterior execução por um outro algoritmo */
void reset_parents()
{
    parent.resize(0);
    parent.resize(network.n);
    for (int i = 0; i < dist.size(); i++)
        parent[i].resize(network.n);
}


/* destroi dados da matrix de distâncias e da matrix de 
predecessores. reconstroi ambas */
void reset_stats()
{
    reset_distances();
    reset_parents();
}


/* a partir de um ficheiro passado como argumento, constroi a rede
do sistema para execução da simulação e do algoritmo    */
void build_network(char filename[])
{
    FILE* fp = fopen(filename, "r");
    int counter = -1;

    int nodes, reader;
    int source, target, width, length;

    std::vector<int> node_aux;
    std::vector<Edge> edge_list;
    std::vector<Node> node_list;

    while (fscanf(fp, "%d,", &reader) == 1)
    {
        if (counter == -1)
            nodes = reader;
        else
        {
            switch (counter)
            {
            case 0:
                source = reader;
                break;
            case 1:
                target = reader;
                break;
            case 2:
                width = reader;
                break;
            case 3:
                length = reader;
                break;
            default:
                break;
            }
        }

        counter++;
        if (counter % 4 == 0 && counter != 0)
        {
            Edge e = { source, target, width, length };
            edge_list.push_back(e);
            counter = 0;
        }
    }

    // reserve spaces for the real nodes of the network
    for (int i = 0; i < MAX_NODES; i++)
        node_aux.push_back(-1);

    // verifica que n�s tem de criar
    for (int i = 0; i < edge_list.size(); i++)
    {
        int t_source = edge_list[i].source;
        int t_target = edge_list[i].target;

        node_aux[t_source] = t_source;
        node_aux[t_target] = t_target;
    }

    // created definitive nodes
    for (int i = 0; i < node_aux.size(); i++)
        if (node_aux[i] != -1)
        {
            Node u;
            u.id = i;
            node_list.push_back(u);
        }

    // run all edges of the network
    // to create the in-neighbours and the out-neighbours of the found nodes
    for (int i = 0; i < edge_list.size(); i++)
    {
        int t_source = edge_list[i].source;
        int t_target = edge_list[i].target;
        int t_length = edge_list[i].length;
        int t_width = edge_list[i].width;

        Edge e = {
            t_source,
            t_target,
            t_width,
            t_length
        };

        // add edge as out-edge at the source
        node_list[t_source].out_edges.push_back(e);

        // add edge as in_edge at the target
        node_list[t_target].in_edges.push_back(e);
    }

    // defines network number of nodes
    network.n = nodes;

    // add the builded nodes to the program network
    network.nodes = node_list;

    // reset distances + parents
    reset_stats();
}


/* imprime dados da matrix de predecessores. por defeito
inclui nós não visitados com -1 no seu predecessor */
void print_parent()
{
    for (int i = 0; i < parent.size(); i++)
    {
        printf("\n");
        for (int j = 0; j < parent[i].size(); j++)
            printf("\t %d", parent[i][j]);
    }
    printf("\n");
}


/*  durante o programa, quando um nó não é visitado, utilizamos
valores por defeito (999; 999) nos pares largura e distância (conforme)
seja mais conveniente para o algoritmo em questão saber tomar descisões.
neste método fazemos a uniformização dos nós não visitados para 
melhor interpretação dos dados  */
void fix_distances()
{
    for (int i = 0; i < network.n; i++)
        for (int j = 0; j < network.n; j++)
            if (sw && dist[i][j].width == EDGE_MIN_WIDTH && dist[i][j].length == EDGE_MAX_LENGTH)
                dist[i][j].width = EDGE_MAX_WIDTH;          
}


/* obtém o nó predecessor de forma recursiva */
void get_parent(int row, int col)
{
    if (parent[row][col] != -1)
    {
        printf(" -> %d", parent[row][col]); 
        if (parent[row][col] != row)
            get_parent(row, parent[row][col]);
    }
}


/* deprecado. era utilizada para obter os predecessores no algoritmo de SW
mas a matrix ninja_parent foi refinada para dentro da matrix parent */
void get_ninja_parent(int node, int row, int col)
{
    if (ninja_parent[node][row][col] != -1)
    {
        printf("\n in");
        printf(" -> %d", ninja_parent[node][row][col]); 
        if (ninja_parent[node][row][col] != row)
            get_ninja_parent(node, ninja_parent[node][row][col], row);
    }
}


/* obter caminhos encontrados. ignora
caminho não existentes/nós não visitados*/
void get_routes()
{
    for (int i = 0; i < network.n; i++)
        for (int j = 0; j < network.n; j++)
            if (dist[i][j].width != EDGE_MAX_LENGTH && dist[i][j].length != EDGE_MAX_WIDTH)
            {
                printf("\n\t(%2d,%3d) : %d", dist[i][j].width, dist[i][j].length, i);
                get_parent(j, i);
            }
}


/* utilizado no final do algoritmo/simulação para refinar e uniformizar matrix
de distâncias e matrix de predecessores. olhando à flag <data>, pode ou não 
criar os ficheiros das CCDFs*/
void output(std::string algoritmo)
{
    fix_distances();
    print_distances();
    
    // algoritmo falha a imprimir os predecessores
    if (algoritmo != "algoritmo_sw" && algoritmo != "algoritmo_ws")
        get_routes();

    if (data)
        ccdf(algoritmo);

    reset_stats();
}


/* adiciona um novo evento ao calendario respeitando o tempo
de execução do evento  */
void add_new_event(Event e)
{
    bool spoted = false;

    // se � menor que o atual, insere nesta posi��o
    // as outras posi��es devem andar uma posi��o para a frente
    for (int i = 0; i < calendar.list.size(); i++)
        if (e.time < calendar.list[i].time)
        {
            calendar.list.insert(calendar.list.begin() + i, e);
            spoted = true;
            break;
        }

    // se n�o encontrou, este evento entra para o final da lista
    if (!spoted)
        calendar.list.push_back(e);

}


// awake a node happens when the node has new data
// and retrieves it to all of his in-neighbours
// the messages are scheduled in the calendar
void awake_node(int node_id, auto start)
{
    double time;

    //the node instantiates the (width, length) to itself
    dist[node_id][node_id].width = EDGE_MAX_WIDTH;
    dist[node_id][node_id].length = 0;

    // for each in-neighbour add new event to the calendar
    for (int i = 0; i < network.nodes[node_id].in_edges.size(); i++)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // get event time
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = now - start;
        time = elapsed.count() + get_random() + TIME_CONST;

        Event e =
        {
            time,
            network.nodes[node_id].in_edges[i].target,
            network.nodes[node_id].in_edges[i].source,
            network.nodes[node_id].in_edges[i].width,
            network.nodes[node_id].in_edges[i].length
        };

        add_new_event(e);
    }

    // debug
    if (debug)
    {
        print_calendar(calendar);
    }
}


// for every node left on the queue, get the one 
// with less distance to the source and return its index
// doesn't return the node, returns the queue index
int ws_minimum(std::vector<int> queue, int index)
{
    int current_length = EDGE_MAX_LENGTH;
    int current_width = EDGE_MAX_WIDTH;
    int current_index = 0;

    for (int i = 0; i < queue.size(); i++)
    {
        if (current_length > dist[queue[i]][index].length ||
            (current_length == dist[queue[i]][index].length && current_width < dist[queue[i]][index].width))
        {
            current_length = dist[queue[i]][index].length;
            current_width = dist[queue[i]][index].width;
            current_index = i;
        }
    }
    return current_index;
}


// for every node left on the queue, get the one 
// with biggest width to the source and return its index
// doesn't return the node, returns the queue index
int sw_minimum(std::vector<int> queue, int index)
{
    int current_length = EDGE_MAX_LENGTH;
    int current_width = 0;
    int current_index = 0;

    for (int i = 0; i < queue.size(); i++)
    {

        if (current_width < dist[queue[i]][index].width ||
            (current_width == dist[queue[i]][index].width && current_length > dist[queue[i]][index].length))
        {
            current_length = dist[queue[i]][index].length;
            current_width = dist[queue[i]][index].width;
            current_index = i;
        }
    }

    return current_index;
}


// comparator used by Dijkstra and Simulator
// to update the distances based on the widest of the shortest
void widest_shortest(int i, int u, int v, int du, int dv, int luv, int w, int wuv, auto start_time, bool simulation)
{
    if (debug)
        printf("\n[%d -> %d] : %d vs %d + %d : min { %d ; %d }", v, i, dv, du, luv, w, wuv);

    // se a dist�ncia encontrada � menor, atualiza
    // se a dist�ncia � igual � atual, temos de ver se a largura melhora
    if (dv > du + luv || dv == du + luv && w < wuv)
    {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start_time;

        dist[v][i].length = du + luv;
        dist[v][i].width = w < wuv ? w : wuv;
        dist[v][i].it = global_iteration;   
        dist[v][i].time = elapsed.count();

        if (simulation) 
            parent[i][v] = u;
        else
            parent[i][v] = u;

        if (debug)
            printf("\n\tAtualizei a posição (%d,%d) com (%d, %d) tem parent = %d", v, i, w, du + luv, v);

        //adiciona novo evento ao calendario
        if (simulation)
            awake_node(v, start_time);
    }
}


// comparator used by Dijkstra and Simulator
// to update the distances based on the shortest of the widest
void shortest_widest(int i, int u, int v, int du, int dv, int luv, int w, int wvi, auto start_time, bool simulation)
{
    if (debug)
        printf("\n\t%d -> %d : (%d vs %d + %d) : {%d vs %d}", u, v, dv, du, luv, wvi, w);

    // se a largura encontrada � maior, atualiza
    // se a largura encontra � igual, v� se a dist�ncia � mais curta
    if (w > wvi || w == wvi && dv > du + luv)
    {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start_time;

        dist[v][i].width = w;
        dist[v][i].length = du + luv;
        dist[v][i].it = global_iteration;
        dist[v][i].time = elapsed.count();

        if (simulation) 
            parent[i][v] = u;
        else
            parent[i][v] = u;
        
        if (debug)
            printf("\n\tAtualizei a posição (%d,%d) com (%d, %d) tem parent = %d", v, i, w, du + luv, v);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (simulation)
            awake_node(v, start_time);
    }

    if (debug)
        print_distances();
}


// simulator for vectoring protocol
// interactive mode simulating routing between source and destination
void simulator_sd(Graph G, int source, int destination)
{
    printf("\nSimulator source [%d] to destintion [%d] ", source, destination);
    
    // start counting time for simulation
    auto start = std::chrono::high_resolution_clock::now();

    if (sw)
        for (int i = 0; i < dist.size(); i++)
            for (int j = 0; j < dist[i].size(); j++)
                dist[i][j].width = 0;


    for (int i = 0; i < parent.size(); i++)
        for (int j = 0; j < parent[i].size(); j++)
            parent[i][j] = -1;


    // awake node
    awake_node(destination, start);

    //processa evento 
    while (calendar.list.size())
    {
        global_iteration++;

        int u = calendar.list[0].event.source;              // sender node
        int v = calendar.list[0].event.target;              // receiver node

        int du = dist[u][destination].length;                         // distance from node (u) to the current announced node
        int dv = dist[v][destination].length;                         // distance from nove (v) to the current announced node
        int luv = calendar.list[0].event.length;            // distance of the already found path

        int wui = dist[u][destination].width;                         // width from current node (u) to destination node (i)
        int wvi = dist[v][destination].width;                         // width from node (v) to destination node (i)
        int wuv = calendar.list[0].event.width;             // width of the current edge which is gonna be part of the new path from (v) to (i)

        int w = wui < wuv ? wui : wuv;

        if (sw)
            shortest_widest(destination, u, v, du, dv, luv, w, wvi, start, true);
        else
            widest_shortest(destination, u, v, du, dv, luv, w, wvi, start, true);

        // removes event from the list
        // since its fifo, remove from the head        
        calendar.list.erase(calendar.list.begin());
    }

    print_distances(source, destination);
}


// simulator for vectoring protocol
// each node is awaken at the time
void simulator(Graph G)
{
    printf("\nSimulator");
    std::string algoritmo = sw ? "simulator_sw" : "simulator_ws";

    // start counting time for simulation
    auto start = std::chrono::high_resolution_clock::now();

    // if is shortest-widest we must fix the max width
    // to zero, so that anything found should potencially an improve
    if (sw)
        for (int i = 0; i < dist.size(); i++)
            for (int j = 0; j < dist[i].size(); j++)
                dist[i][j].width = 0;


    // set all predecessors as -1
    for (int i = 0; i < parent.size(); i++)
        for (int j = 0; j < parent[i].size(); j++)
            parent[i][j] = -1;


    // for each node 
    // add to calendar
    // look at each node edges, add create new events
    for (int i = network.n - 1; i >= 0; i--)
    {
        // awake node
        awake_node(network.nodes[i].id, start);

        //processa evento 
        while (calendar.list.size())
        {
            global_iteration++;

            int u = calendar.list[0].event.source;              // sender node
            int v = calendar.list[0].event.target;              // receiver node

            int du = dist[u][i].length;                         // distance from node (u) to the current announced node
            int dv = dist[v][i].length;                         // distance from nove (v) to the current announced node
            int luv = calendar.list[0].event.length;            // distance of the already found path

            int wui = dist[u][i].width;                         // width from current node (u) to destination node (i)
            int wvi = dist[v][i].width;                         // width from node (v) to destination node (i)
            int wuv = calendar.list[0].event.width;             // width of the current edge which is gonna be part of the new path from (v) to (i)

            int w = wui < wuv ? wui : wuv;

            if (sw)
                shortest_widest(i, u, v, du, dv, luv, w, wvi, start, true);
            else
                widest_shortest(i, u, v, du, dv, luv, w, wvi, start, true);

            // removes event from the list
            // since its fifo, remove from the head        
            calendar.list.erase(calendar.list.begin());
        }
    }
    output(algoritmo);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\n\n\tElapsed Time: " << elapsed.count() << " seconds" << std::endl;
}


// Dijkstra algorithm
// implementing a destination to source
void dijkstra_sd(Graph network, int source, int destination)
{
    printf("\nDijkstra source [%d] to destination [%d] ", source, destination);
    std::vector<int> queue;

    // start counting time for simulation
    auto start = std::chrono::high_resolution_clock::now();

    // initialization
    // no one has parents at the begin
    // lenght and width are infinite
    // only instantiates vector relative to the node that is destination
    for (int i = 0; i < network.n; i++)
    {
        Info dummy = { EDGE_MAX_WIDTH, EDGE_MAX_LENGTH, 0, 0 };
        
        queue.push_back(network.nodes[i].id);
        parent[destination][i] = (-1);
        dist[destination].push_back(dummy);
    }

    // if is shortest-widest we must fix the max width
    // to zero, so that anything found should potencially an improve
    if (sw)
        for (int i = 0; i < network.n; i++)
            dist[i][destination].width = 0;


    // for itself, width is max and length is zero
    dist[destination][destination].width = EDGE_MAX_WIDTH;
    dist[destination][destination].length = 0;


    while (queue.size())
    {
        // obter o nó cuja distância é menor
        // devolve o index da queue onde está o nó, é preciso ir buscá-lo à queue
        int index = sw ? sw_minimum(queue, destination) : ws_minimum(queue, destination);
        int u = queue[index];
        global_iteration++;

        // relaxar cada aresta uv
        for (int i = 0; i < network.nodes[u].in_edges.size(); i++)
        {
            int v = network.nodes[u].in_edges[i].source;            // node (v) which is going to be analised

            int dw = dist[v][destination].width;                    // width of the current path analised
            int dv = dist[v][destination].length;                   // length dv (from v -> destination)
            int du = dist[u][destination].length;                   // length du (from u -> destination)

            int luv = network.nodes[u].in_edges[i].length;          // length luv from u -> v
            int wuv = network.nodes[u].in_edges[i].width;           // width of the current edge analised
            wuv = wuv < dist[u][destination].width ?                // minimization { w(u); w(uv) }
                wuv : dist[u][destination].width;

            if (sw)
                shortest_widest(destination, u, v, du, dv, luv, wuv, dw, start, false);
            else
                widest_shortest(destination, u, v, du, dv, luv, dw, wuv, start, false);
        }

        //remove nó da queue
        queue.erase(queue.begin() + index);
    }
    print_distances(source, destination);
}


// Dijkstra algorithm
// implementing a destination to all sources search
void dijkstra(Graph network, Node destination, auto start)
{
    Info dummy = { EDGE_MAX_WIDTH, EDGE_MAX_LENGTH, 0, 0 };
    std::vector<int> queue;

    // initialization
    // no one has parents at the begin
    // lenght and width are infinite
    // only instantiates vector relative to the node that is destination
    for (int i = 0; i < network.n; i++)
    {
        queue.push_back(network.nodes[i].id);
        parent[destination.id][i] = (-1);
        dist[destination.id].push_back(dummy);
    }

    if (sw)
        for (int i = 0; i < network.n; i++)
            dist[i][destination.id].width = 0;


    // for itself, width is max and length is zero
    dist[destination.id][destination.id].width = EDGE_MAX_WIDTH;
    dist[destination.id][destination.id].length = 0;


    int k = 0;
    while (queue.size())
    {
        // obter o n� cuja dist�ncia � menor
        // devolve o index em que est� o n�, � preciso ir busc�-lo
        int index = sw ? sw_minimum(queue, destination.id) : ws_minimum(queue, destination.id);
        int u = queue[index];
        global_iteration++;

        // relaxar cada aresta uv
        for (int i = 0; i < network.nodes[u].in_edges.size(); i++)
        {
            int v = network.nodes[u].in_edges[i].source;            // node (v) which is going to be analised

            int dw = dist[v][destination.id].width;                 // width of the current path analised
            int dv = dist[v][destination.id].length;                // length dv (from v -> destination)
            int du = dist[u][destination.id].length;                // length du (from u -> destination)

            int luv = network.nodes[u].in_edges[i].length;          // length luv from u -> v
            int wuv = network.nodes[u].in_edges[i].width;           // width of the current edge analised
            wuv = wuv < dist[u][destination.id].width ?             // minimization { w(u); w(uv) }
                wuv : dist[u][destination.id].width;

            if (sw)
                shortest_widest(destination.id, u, v, du, dv, luv, wuv, dw, start, false);
            else
                widest_shortest(destination.id, u, v, du, dv, luv, dw, wuv, start, false);
        }

        // remove analised node from queue
        queue.erase(queue.begin() + index);
    }
}


// run Dijkstra for each node as destination
void dijkstra(Graph network)
{
    printf("\nDijkstra");
    std::string algoritmo = sw ? "dijkstra_sw" : "dijkstra_ws";

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = network.nodes.size() - 1; i >= 0; i--)
        dijkstra(network, network.nodes[i], start);
    output(algoritmo);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\n\n\tElapsed Time: " << elapsed.count() << " seconds" << std::endl;
}


void algoritmo(Graph network, Node destination, auto start)
{
    Info dummy = { 0, EDGE_MAX_LENGTH, 0, 0 };
    std::vector<int> queue;

    // create ninja-parent
    ninja_parent.resize(network.n);
    for (int i = 0; i < dist.size(); i++)
    {
        ninja_parent[i].resize(network.n);
        for(int j=0; j<ninja_parent[i].size(); j++)
            ninja_parent[i][j].resize(network.n);
    }

    for (int i = 0; i < ninja_parent.size(); i++)
        for (int j = 0; j < ninja_parent[i].size(); j++)
            for(int k=0; k < ninja_parent[i][j].size(); k++)
                ninja_parent[i][j][k] = -1;


    // initialization of the queue
    for (int i = 0; i < network.n; i++)
        queue.push_back(network.nodes[i].id);

    cd.resize(network.n);
    for (int i = 0; i < network.n; i++)
        cd[i].resize(network.n);

    for (int i = 0; i < network.n; i++)
        dist[i][destination.id].width = 0;    
     
    // for itself, width is max and length is zero
    dist[destination.id][destination.id].width = EDGE_MAX_WIDTH;
    dist[destination.id][destination.id].length = 0;


    while (queue.size())
    {

        // obter o nó cuja distância é menor
        // devolve o index em que está o nó, é preciso ir buscá-lo
        int index = sw_minimum(queue, destination.id);
        int u = queue[index];
        

        // relaxar cada aresta uv
        for (int i = 0; i < network.nodes[u].in_edges.size(); i++)
        {
            int v = network.nodes[u].in_edges[i].source;            // node (v) which is going to be analised

            int dw = dist[v][destination.id].width;                 // width of the current path analised
            int dv = dist[v][destination.id].length;                // length dv (from v -> destination)
            int du = dist[u][destination.id].length;                // length du (from u -> destination)

            int luv = network.nodes[u].in_edges[i].length;          // length luv from u -> v
            int wuv = network.nodes[u].in_edges[i].width;           // width of the current edge analised
            wuv = wuv < dist[u][destination.id].width ?             // minimization { w(u); w(uv) }
                wuv : dist[u][destination.id].width;

           
            if (cd[u][destination.id].size() < 2)
            {

                // analisador SW default
                if (dw < wuv || dw == wuv && dv > du + luv)
                {
                    auto finish = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> elapsed = finish - start;

                    dist[v][destination.id].width = wuv;
                    dist[v][destination.id].length = du + luv;
                    dist[v][destination.id].it = global_iteration;
                    dist[v][destination.id].time = elapsed.count();

                    // atualiza pai
                    // não preciso de atualizar dos dois lados, só preciso de usar 
                    // corretamento o ninja_parent mas esta parte não está a funcionar
                    // parece-me que a iteração do proxumo destination às vezes reescreve por cima
                    parent[destination.id][v] = u;
                    ninja_parent[destination.id][u][v] = destination.id;
                    
                    // adiciona aresta encontrada como futuro par candidato
                    candidate_pair ncp = { wuv, du+luv, u, 0};
                    cd[v][destination.id].push_back(ncp);

                    // idk
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            else
            {
                int width = EDGE_MIN_WIDTH;
                int length = EDGE_MAX_LENGTH;
                int index = 0;

                // tenho de percorrer cada par dominante e ver qual é escolhido
                for (int i=0; i<cd[u][destination.id].size(); i++)
                {

                    // operação de extensão da arestas atual 
                    // com cada um dos pares candidatos
                    // minima a largura e soma as distâncias
                    int candidate_width = cd[u][destination.id][i].width < wuv ? cd[u][destination.id][i].width : wuv;  
                    int candidate_length = cd[u][destination.id][i].length + luv;                                      


                    // which is the optimal path?
                    if (candidate_width > width ||
                        candidate_width == width && candidate_length < length)
                    {
                        width = candidate_width;
                        length = candidate_length;
                        index = i;
                    }
                }

                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsed = finish - start;

                // atualiza distância encontrada
                dist[v][destination.id].width = width;
                dist[v][destination.id].length = length;
                dist[v][destination.id].it = global_iteration;
                dist[v][destination.id].time = elapsed.count();

                // atualiza pai -> deprecated
                parent[destination.id][v] = u;


                // adiciona par candidato
                // no futuro vai ser apresentado a um nó
                // e eventualmente pode ser escolhido de entre os seus pares
                candidate_pair ncp = { width, length, 0, 0 };
                cd[v][destination.id].push_back(ncp);


                // não preciso de atualizar dos dois lados, só preciso de usar 
                // corretamento o ninja_parent mas esta parte não está a funcionar
                // parece-me que a iteração do proxumo destination às vezes reescreve por cima
                ninja_parent[destination.id][v][u];
                ninja_parent[destination.id][destination.id][u] = cd[u][destination.id][i].label;
                ninja_parent[destination.id][destination.id][v] = u;

                //idk
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // remove analised node from queue
        queue.erase(queue.begin() + index);
        global_iteration++;
    }
}

void algoritmo()
{
    printf("\nAlgorithm");
    std::string algorithm = sw ? "algoritmo_sw" : "algoritmo_ws";

    auto start = std::chrono::high_resolution_clock::now();
    for (int k = network.nodes.size() - 1; k >= 0; k--)
    {
        algoritmo(network, network.nodes[k], start);
        for (int i=0; i<network.n; i++)
            parent[k][i] = ninja_parent[k][k][i];
    }
    output(algorithm);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\n\n\tElapsed Time: " << elapsed.count() << " seconds" << std::endl;   
}

int main() 
{

    char filename[] = "input/project.txt";
    build_network(filename);

    sw = false;
    debug = false;
    data = true;
    
    /* simulator + dijkstra + algoritm */
    printf("\n>>> Widest Shortest");
    simulator(network);
    dijkstra(network); 

    sw = true;

    printf("\n>>> Shortest Widest");
    simulator(network); 
    dijkstra(network); 
    algoritmo();


    /* interactive mode */
    int source, destination; 
    std::cout << "\n\nInput source node: "; 
    std::cin >> source; 

    std::cout << "Input destination node: ";
    std::cin >> destination;

    dijkstra_sd(network, source, destination);
    simulator_sd(network, source, destination);

    printf("\n");
    return 0;
}
