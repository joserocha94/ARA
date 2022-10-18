#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ctime>  
#include <sys/time.h>
#include <chrono> 
#include <thread>
#include <fstream>

#define GRAPH_MAX_SIZE 4
#define MAX_NODES 9999
#define EDGE_MAX_WIDTH 999
#define EDGE_MAX_LENGHT 999

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

    void print()
    {
        printf("\nNetwork in-edges \n");
        for (int i=0; i<nodes.size(); i++)
            for (int j=0; j <nodes[i].in_edges.size(); j++)
                std::cout << "[" 
                << nodes[i].id  << "] (" 
                << nodes[i].in_edges[j].source << "," 
                << nodes[i].in_edges[j].target << ")" 
                << std::endl;
    }

    void print_out()
    {
        printf("\nNetwork out-edges \n");
        for (int i=0; i<nodes.size(); i++)
            for (int j=0; j <nodes[i].out_edges.size(); j++)
                std::cout << "[" 
                << nodes[i].id  << "] (" 
                << nodes[i].out_edges[j].source << "," 
                << nodes[i].out_edges[j].target << ")" 
                << std::endl;
    }
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

std::vector<std::vector<std::pair<int,int>>> dist;
std::vector<std::vector<int>> parent;

Graph network; 
Calendar calendar;

bool debug = false;

// uniform distribution between 0 and 1
// used to increment the event random delay
double get_random()
{
    return ((double) rand() / (RAND_MAX));
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
void print_distances(std::vector<std::vector<std::pair<int,int>>> dist)
{
    for (int i=0; i<dist.size(); i++)
    {
        printf("\n");
        for (int j=0; j<GRAPH_MAX_SIZE; j++)
            printf("\t(%4d; %4d)", dist[i][j].first, dist[i][j].second); 
    }
}


// from the input file, builds the program network
// reads all edges, from the edges get all network nodes
// assign in_edges and out_edges to each node
void build_network(char filename[])
{
    
    FILE *fp = fopen(filename, "r");
    int counter = -1;

    int nodes, reader;
    int source, target, width, length;

    std::vector<int> node_aux;
    std::vector<Edge> edge_list;
    std::vector<Node> node_list;

    while(fscanf(fp, "%d,", &reader) == 1) 
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

    // verifica que nós tem de criar
    for (int i=0; i<edge_list.size(); i++)
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
    for (int i=0; i<edge_list.size(); i++)
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

    // matrix with pairs width-length
    dist.resize(GRAPH_MAX_SIZE);
    for (int i = 0; i < dist.size(); i++)
        for (int j = 0; j < GRAPH_MAX_SIZE; j++)
            dist[i].push_back(std::make_pair(EDGE_MAX_WIDTH, EDGE_MAX_LENGHT));

    // matrix with parents for path search
    parent.resize(GRAPH_MAX_SIZE);
    for (int i = 0; i < dist.size(); i++)
        parent[i].resize(GRAPH_MAX_SIZE);
}


// for every node left on the queue, get the one 
// with less distance to the source and return its index
// doesn't return the node, returns the queue index
int ws_minimum(std::vector<int> queue, std::vector<std::pair<int,int>> distances)
{
    int current_length = EDGE_MAX_LENGHT;
    int current_width = EDGE_MAX_WIDTH;
    int current_index = 0;

    for (int i=0; i<queue.size(); i++) 
    {
        if (current_length > distances[queue[i]].second || 
           (current_length == distances[queue[i]].second && current_width < distances[queue[i]].first)) 
        {
            current_length = distances[queue[i]].second;
            current_width = distances[queue[i]].first;
            current_index = i;
        }
    }
    return current_index;
}


// used to print the predecessor nodes 
// along each path that has been found
void get_parent(std::vector<int> parent, int start)
{
    if (parent[start] != -1)
    {
        printf(" <- %d", parent[start]);
        get_parent(parent, parent[start]);
    }
}


// when a protocol/algorithm as finished the paths
// may be found at parent matrix and be printed
// ignores inexistent paths
void get_routes(std::vector<std::pair<int,int>> q_distance, std::vector<int> q_parent)
{
    for (int i = 0; i < network.n; i++)
        if (q_distance[i].first != EDGE_MAX_LENGHT && q_distance[i].second != EDGE_MAX_WIDTH)
        {
            printf("\n\t(%2d,%3d) : %d",  q_distance[i].first, q_distance[i].second, i);
            get_parent(q_parent, i);
        }
}


// adds a new event to the calendar
// fifo order, due to the event time
void add_new_event(Event e)
{    
    bool spoted = false;

    // se é menor que o atual, insere nesta posição
    // as outras posições devem andar uma posição para a frente
    for (int i = 0; i < calendar.list.size(); i++) 
        if (e.time < calendar.list[i].time)
        {
            calendar.list.insert(calendar.list.begin() + i, e);
            spoted = true;
            break;
        }

    // se não encontrou, este evento entra para o final da lista
    if (!spoted)
        calendar.list.push_back(e);

}


// Dijkstra algorithm
// implementing a source to all search
// pair<int,int> where [first=width]; [second=lenght]
void dijkstra(Graph g, Node s)
{
    printf("\nDijkstra from %d \n", s.id);

    std::vector<int> q;
    std::vector<int> q_parent;
    std::vector<std::pair<int,int>> q_distance;

    // initialization
    // no one has parents at the begin
    // lenght and width are infinite * to do: check this infinitive width 
    for (int i=0; i<g.n; i++)
    {
        q.push_back(g.nodes[i].id);
        q_parent.push_back(-1);
        q_distance.push_back(std::make_pair(EDGE_MAX_WIDTH, EDGE_MAX_LENGHT));
    }

    // for itself, widths and lenghts are zero
    q_distance[s.id].first = 0; 
    q_distance[s.id].second = 0;

    while (q.size())
    {
        // obter o nó cuja distância é menor
        // devolve o index em que está o nó, é preciso ir buscá-lo
        int index = ws_minimum(q, q_distance); 
        int u = q[index];

        // relaxar cada aresta uv
        for (int i=0; i<g.nodes[u].out_edges.size(); i++)
        {
            int v = g.nodes[u].out_edges[i].target;
            int uv_lenght = g.nodes[u].out_edges[i].length;
            int uv_width = g.nodes[u].out_edges[i].width;

            // se lenght mais curta do que está nas distancias, actualiza
            // fica com o menor valor para a width
            if (q_distance[v].second > q_distance[u].second + uv_lenght)
            {
                q_distance[v].second = q_distance[u].second + uv_lenght;
                q_distance[v].first = q_distance[u].first < uv_width && q_parent[u] != -1 ? q_distance[u].first : uv_width;
                q_parent[v] = u;
            }
        }

        //remove nó da queue
        q.erase(q.begin() + index);
    }

    // iteração terminou
    // show path for each node
    get_routes(q_distance, q_parent);
}


// awake a node happens when the node has new data
// and retrieves it to all of his in-neighbours
// the messages are scheduled in the calendar
void awake_node(int node_id, auto start)
{        
    double time;

    //the node instantiates the (width, length) to itself
    dist[node_id][node_id].first = EDGE_MAX_WIDTH;
    dist[node_id][node_id].second = 0;

    // for each in-neighbour add new event to the calendar
    for (int i = 0; i < network.nodes[node_id].in_edges.size(); i++)
    {
        // get event time
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = now - start;
        time = elapsed.count() + get_random();

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
        print_calendar(calendar);
}


// comparator used by Dijkstra and Simulator
// to update the distances based on the widest of the shortest
void widest_shortest(int i, int u, int v, int du, int dv, int luv, int w, int wuv, auto start_time)
{
    // se a distância encontrada é menor, atualiza
    // se a distância é igual à atual, temos de ver se a largura melhora
    if (dv > du + luv || dv == du + luv && w < wuv)
    {
        dist[v][i].second = du + luv;
        dist[v][i].first = w < wuv ? w : wuv;
        parent[v][u] = u;

        //adiciona novo evento ao calendario
        awake_node(v, start_time);
    }
}


// comparator used by Dijkstra and Simulator
// to update the distances based on the shortest of the widest
void shortest_widest(int i, int u, int v, int du, int dv, int luv, int w, int wvi, auto start_time)
{
    // in shortest_widest we must change default width 
    if (wvi == EDGE_MAX_WIDTH)
        wvi = -wvi;

    if (debug)
        printf("\n\t%d -> %d : (%d vs %d + %d) : {%d vs %d}",  
            u, v, dv, du, luv, wvi, w, i);

    // se a largura encontrada é maior, atualiza
    // se a largura encontra é igual, vê se a distância é mais curta
    if (w > wvi || w == wvi && dv > du + luv)
    {
        dist[v][i].first = w;
        dist[v][i].second = du + luv;
        parent[v][u] = u;

        if (debug)
            printf("\n\n\tAtualizei a posição (%d,%d) e acordei o %d", v, i, v);
        
        awake_node(v, start_time);
    }
}


// simulator for vectoring protocol
// each node is awaken at the time
void simulator(Graph G)
{
    // start counting time for simulation
    auto start = std::chrono::high_resolution_clock::now();

    // for each node 
    // add to calendar
    // look at each node edges, add create new events
    for (int i = network.n-1; i >= 0; i--)
    {
        // awake node
        awake_node(network.nodes[i].id, start);

        //processa evento 
        while (calendar.list.size())
        {
            if (debug)
                printf("\n\nVai processar o próximo evento");

            int u = calendar.list[0].event.source;      // sender node
            int v = calendar.list[0].event.target;      // receiver node

            int du = dist[u][i].second;                 // distance from node (u) to the current announced node
            int dv = dist[v][i].second;                 // distance from nove (v) to the current announced node
            int luv = calendar.list[0].event.length;    // distance of the already found path

            int wui = dist[u][i].first;                 // width from current node (u) to destination node (i)
            int wvi = dist[v][i].first;                 // width from node (v) to destination node (i)
            int wuv = calendar.list[0].event.width;     // width of the current edge which is gonna be part of the new path from (v) to (i)

            int w = wui < wuv ? wui : wuv;
           
            widest_shortest(i, u, v, du, dv, luv, w, wvi, start);   //wvi was wuv
            //shortest_widest(i, u, v, du, dv, luv, w, wvi, start);       

            if (debug)
                print_distances(dist);

            // removes event from the list
            // since its fifo, remove from the head        
            calendar.list.erase(calendar.list.begin());
        }
    }

    print_distances(dist);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\nElapsed Time: " << elapsed.count() << " seconds" << std::endl;
}


int main() // u0, w1, v2, x3
{
    print_distances(dist);
    printf("\n");

    char filename [] = "network.txt";
    build_network(filename);

    simulator(network);

    printf("\n");
    return 0;
}
