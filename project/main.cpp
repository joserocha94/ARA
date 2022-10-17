#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ctime>  
#include <sys/time.h>
#include <chrono> 
#include <thread>



#define GRAPH_MAX_SIZE 4
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

Graph G; 
Calendar C;


double get_random()
{
    return ((double) rand() / (RAND_MAX));
}

void print_calendar(Calendar c)
{
    printf("\n");
    for (int i=0; i<c.list.size(); i++)
        std::cout << "\tevento " << i << ": ("
        << c.list[i].event.source << " > "
        << c.list[i].event.target << ") : "
        << c.list[i].time 
        << std::endl;
}

void print_distances(std::vector<std::vector<std::pair<int,int>>> dist)
{
    for (int i=0; i<dist.size(); i++)
    {
        printf("\n");
        for (int j=0; j<GRAPH_MAX_SIZE; j++)
            printf("\t(%4d; %4d)", dist[i][j].first, dist[i][j].second); 
    }
}

// matrix with pairs width-length
void init()
{
    dist.resize(GRAPH_MAX_SIZE);
    for (int i=0; i < dist.size(); i++)
        for (int j=0; j < GRAPH_MAX_SIZE; j++)
            dist[i].push_back(std::make_pair(EDGE_MAX_WIDTH, EDGE_MAX_LENGHT));

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

void get_parent(std::vector<int> parent, int start)
{
    if (parent[start] != -1)
    {
        printf(" <- %d", parent[start]);
        get_parent(parent, parent[start]);
    }
}

void get_routes(std::vector<std::pair<int,int>> q_distance, std::vector<int> q_parent)
{
    for (int i=0; i<G.n; i++)
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
    for (int i = 0; i < C.list.size(); i++) 
        if (e.time < C.list[i].time)
        {
            C.list.insert(C.list.begin() + i, e);
            spoted = true;
            break;
        }

    // se não encontrou, este evento entra para o final da lista
    if (!spoted)
        C.list.push_back(e);

    // debug
    print_calendar(C);
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

void simulator(Graph G)
{

    auto start = std::chrono::high_resolution_clock::now();
    double time;

    // for each node 
    // add to calendar
    // look at each node edges, add create new events
    for (int i = G.n-1; i == G.n-1; i--)
    {
        printf("\nAcordar nó %d com i=%d", G.nodes[i].id, i);

        //para cada out-neighbour, adicionar uma mensagem ao calendario
        for (int j=0; j<G.nodes[i].in_edges.size(); j++)
        {

            // get event time
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = now - start;
            time = elapsed.count() + get_random();

            Event e = 
            {
                time,
                G.nodes[i].in_edges[j].target,  
                G.nodes[i].in_edges[j].source, 
                G.nodes[i].in_edges[j].width, 
                G.nodes[i].in_edges[j].length
            };

            add_new_event(e);
        
            printf("Evento %d->%d - [%d,%d] with time %f",                 
                e.event.source,
                e.event.target,
                e.event.width,
                e.event.length, 
                e.time);
        }

        printf("\nadicionar o novo");
        Event y = {0.0, 1, 2, 2, 2};
        add_new_event(y);

        //processa evento 
        while (C.list.size())
        {
            printf("\nVai processar o próximo evento");
            C.list.erase(C.list.begin());
        }
    }

    printf("\n");

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "\nElapsed Time: " << elapsed.count() << " seconds" << std::endl;
}

int main() // u0, w1, v2, x3
{
    init();
    print_distances(dist);
    printf("\n");

    Edge e1 = {0, 1, 5, 1};
    Edge e2 = {0, 2, 10, 2};
    Edge e3 = {2, 1, 20, 4};
    Edge e4 = {1, 3, 20, 1};
    Edge e5 = {2, 3, 10, 2};

    Node u; u.id = 0;
    Node w; w.id = 1;
    Node v; v.id = 2;
    Node x; x.id = 3;

    u.out_edges.push_back(e1);
    u.out_edges.push_back(e2);
    v.out_edges.push_back(e3);
    w.out_edges.push_back(e4);
    v.out_edges.push_back(e5);

    v.in_edges.push_back(e2);
    w.in_edges.push_back(e1);
    w.in_edges.push_back(e3);
    x.in_edges.push_back(e4);
    x.in_edges.push_back(e5);
    
    G.n = 4;
    G.nodes.push_back(u);
    G.nodes.push_back(w);
    G.nodes.push_back(v);
    G.nodes.push_back(x);

    G.print();
    printf("\n");
    G.print_out();

    int current_node = 0;
    int loop = 0;
    
    //dijkstra(G, w);

    simulator(G);
    printf("\n");

    return 0;
}
