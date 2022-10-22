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

std::vector<std::vector<Info>> dist;
std::vector<std::vector<int>> parent;

Graph network;
Calendar calendar;

int global_iteration;
bool debug, sw;

// uniform distribution between 0 and 1
// used to increment the event random delay
double get_random()
{
    return ((double)rand() / (RAND_MAX));
}


// prints the calendar events
// since the calendar is always ordered, the output is also
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


// prints the distances [width; leangth] from each node
// to each other node of the source
// includes the default distances (not visited nodes)
void print_distances(std::vector<std::vector<Info>> dist)
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


// print distances for interactive mode
// distance from a source and a destination
void print_distances(std::vector<std::vector<Info>> dist, int source, int destination)
{
    int width = dist[source][destination].width;
    int length = dist[source][destination].length;
    
    printf("was found with (%2d, %2d)", width, length);
}


// destroy distances matrix
// rebuilds it for another algortihm execution
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


// destroy parent matrix
// rebuilds it for another algortihm execution
void reset_parents()
{
    parent.resize(0);
    parent.resize(network.n);
    for (int i = 0; i < dist.size(); i++)
        parent[i].resize(network.n);
}


// destroys both distances and parent matrix
// rebuilds them for another algorithm execution
void reset_stats()
{
    reset_distances();
    reset_parents();
}


// from the input file, builds the program network
// reads all edges, from the edges get all network nodes
// assign in_edges and out_edges to each node
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


// used to print all the data representing
// the predecessors of each node
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


// used to sinalize nodes that haven't been reached
// put (width;length) with default values changing 
// the ones used for comparations in SW and WS paths
void fix_distances()
{
    for (int i = 0; i < network.n; i++)
        for (int j = 0; j < network.n; j++)
            if (sw && dist[i][j].width == EDGE_MIN_WIDTH && dist[i][j].length == EDGE_MAX_LENGTH)
                dist[i][j].width = EDGE_MAX_WIDTH;          
}


// used to print the predecessor nodes 
// along each path that has been found
void get_parent(int row, int col)
{
    //printf("\n checking parent (%d,%d)\n", row, col);
    if (parent[row][col] != -1 && parent[row][col] != col)
    {
        printf(" <- %d", parent[row][col]); 
        get_parent(row, parent[row][col]);
    }
}


// when a protocol/algorithm as finished the paths
// may be found at parent matrix and be printed
// ignores inexistent paths
// in SW having (0;999) means no route; (999, 0) means target node
void get_routes()
{
    for (int i = 0; i < network.n; i++)
        for (int j = 0; j < network.n; j++)
            if (dist[i][j].width != EDGE_MAX_LENGTH && dist[i][j].length != EDGE_MAX_WIDTH)
            {
                printf("\n\t(%2d,%3d) : %d", dist[i][j].width, dist[i][j].length, j);
                get_parent(i, j);
                printf(" <- %d", i);
            }
}


// adds a new event to the calendar
// fifo order, due to the event time
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
    //if (debug)
        printf("\n[%d -> %d] : %d vs %d + %d : min { %d ; %d }", v, i, dv, du, luv, w, wuv);

    // se a dist�ncia encontrada � menor, atualiza
    // se a dist�ncia � igual � atual, temos de ver se a largura melhora
    if (dv > du + luv || dv == du + luv && w < wuv)
    {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start_time;

        dist[v][i].length = du + luv;
        dist[v][i].width = w < wuv ? w : wuv;
        //parent[i][v] = u;
        parent[v][i] = u;

        dist[v][i].it = global_iteration;   
        dist[v][i].time = elapsed.count();

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
        parent[v][i] = u;

        dist[v][i].it = global_iteration;
        dist[v][i].time = elapsed.count();
        
        if (debug)
            printf("\n\tAtualizei a posição (%d,%d) com (%d, %d) tem parent = %d", v, i, w, du + luv, v);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (simulation)
            awake_node(v, start_time);
    }

    if (debug)
        print_distances(dist);
}


// simulator for vectoring protocol
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
        if (debug)
            printf("\n\nVai processar o pr�ximo evento");

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
    print_distances(dist, source, destination);
}


// simulator for vectoring protocol
// each node is awaken at the time
void simulator(Graph G)
{
    printf("\nSimulator");

    // start counting time for simulation
    auto start = std::chrono::high_resolution_clock::now();


    if (sw)
        for (int i = 0; i < dist.size(); i++)
            for (int j = 0; j < dist[i].size(); j++)
                dist[i][j].width = 0;


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
            if (debug)
                printf("\n\nVai processar o pr�ximo evento");

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

    fix_distances();
    print_distances(dist);
    print_parent();

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\n\tElapsed Time: " << elapsed.count() << " seconds" << std::endl;
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

    if (sw)
        for (int i = 0; i < network.n; i++)
            dist[i][destination].width = 0;


    // for itself, width is max and length is zero
    dist[destination][destination].width = EDGE_MAX_WIDTH;
    dist[destination][destination].length = 0;


    while (queue.size())
    {
        // obter o n� cuja dist�ncia � menor
        // devolve o index em que est� o n�, � preciso ir busc�-lo
        int index = sw ? sw_minimum(queue, destination) : ws_minimum(queue, destination);
        int u = queue[index];
        global_iteration++;

        // relaxar cada aresta uv
        for (int i = 0; i < network.nodes[u].in_edges.size(); i++)
        {
            int v = network.nodes[u].in_edges[i].source;            // node (v) which is going to be analised

            int dw = dist[v][destination].width;                 // width of the current path analised
            int dv = dist[v][destination].length;                // length dv (from v -> destination)
            int du = dist[u][destination].length;                // length du (from u -> destination)

            int luv = network.nodes[u].in_edges[i].length;          // length luv from u -> v
            int wuv = network.nodes[u].in_edges[i].width;           // width of the current edge analised
            wuv = wuv < dist[u][destination].width ?             // minimization { w(u); w(uv) }
                wuv : dist[u][destination].width;

            if (sw)
                shortest_widest(destination, u, v, du, dv, luv, wuv, dw, start, false);
            else
                widest_shortest(destination, u, v, du, dv, luv, dw, wuv, start, false);
        }

        //remove n� da queue
        queue.erase(queue.begin() + index);
    }
    print_distances(dist, source, destination);
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

    if (debug)
    {
        printf("\nDestination %d", destination.id);
        print_distances(dist);
        print_parent();
    }

    int k = 0;
    while (queue.size())
    {
        if (debug)
            printf("\n================================================= k=%d", k);

        // obter o n� cuja dist�ncia � menor
        // devolve o index em que est� o n�, � preciso ir busc�-lo
        int index = sw ? sw_minimum(queue, destination.id) : ws_minimum(queue, destination.id);
        int u = queue[index];
        global_iteration++;

        if (debug)
        {
            printf("\nfound index %d", index);
            printf("\nnode %d", u);
        }

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

            if (debug)
            {
                printf("\nestas devem concatenar e ficar com a menor %d vs %d", wuv, dist[u][destination.id].width);
                printf("\nwidth wuv = %d , dw = %d", wuv, dw);
            }

            if (sw)
                shortest_widest(destination.id, u, v, du, dv, luv, wuv, dw, start, false);
            else
                widest_shortest(destination.id, u, v, du, dv, luv, dw, wuv, start, false);


            if (debug)
            {
                print_distances(dist);
                print_parent();
            }
        }

        //remove n� da queue
        queue.erase(queue.begin() + index);
        k++;

        if (debug)
        {
            printf("\nQUEUE:");
            for (int i = 0; i < queue.size(); i++)
                printf("\t %d", queue[i]);
        }
    }
}


// run Dijkstra for each node as destination
void dijkstra(Graph network)
{
    printf("\nDijkstra");

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = network.nodes.size() - 1; i >= 0; i--)
        dijkstra(network, network.nodes[i], start);

    fix_distances();
    print_distances(dist);
    print_parent();

    // itera��o terminou
    // show path for each node
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\n\tElapsed Time: " << elapsed.count() << " seconds" << std::endl;
}



int main() 
{

    char filename[] = "network.txt";
    build_network(filename);

    debug = sw = false;
    
    printf("\n>>> Widest Shortest");
    //simulator(network); reset_stats();
    dijkstra(network);  reset_stats();
/*
    sw = true;

    printf("\n>>> Shortest Widest");
    simulator(network); reset_stats();
    dijkstra(network);  reset_stats(); 
*/

    //dijkstra_sd(network, 2, 3);
    //simulator_sd(network, 2, 3);
    
    printf("\n");
    return 0;
}
