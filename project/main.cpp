#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ctime>  

#define GRAPH_MAX_SIZE 4
#define EDGE_MAX_DISTANCE 999

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
    Edge event;
    time_t start;
    time_t end;
};

struct Calendar
{
    std::vector<Event> list;
};



void print_calendar(Calendar c)
{
    printf("SCHEDULER:\n");
    for (int i=0; i<c.list.size(); i++)
        std::cout << "       ["
        << c.list[i].event.source << "] >>> ["
        << c.list[i].event.target << "]"
        << std::endl;
}

// for every node left on the queue, get the one 
// with less distance to the source and return its index
// doesn't return the node, returns the queue index
int minimum(std::vector<int> queue, std::vector<int> distances)
{
    int current_distance = EDGE_MAX_DISTANCE;
    int current_index = 0;

    for (int i=0; i<queue.size(); i++) 
    {
        if (current_distance > distances[queue[i]]) {
            current_distance = distances[queue[i]];
            current_index = i;
        }
    }
    return current_index;
}


// attempt to implement Dijkstra algorithm
// priority queue / min heap missing
void dijkstra(Graph g, Node s)
{
    // every node has a distance of 999 (infinite)
    // source node has distance 0
    std::vector<int> distances;
    for (int i=0; i<g.n; i++)
        distances.push_back(EDGE_MAX_DISTANCE);
    distances[s.id] = 0;
    for (int i=0; i<g.n; i++)
        printf("\npos:%d dist:%d", i, distances[i]);

    // add all graph vertices to the queue
    std::vector<int> queue;
    for(int i=0; i<g.n; i++)
        queue.push_back(i);


    // while there are still nodes in the queue
    printf("\n");
    while (queue.size())
    {
        printf("\nelements %d", queue.size());
        queue.pop_back();

        //find node with less distance to source
        int min = minimum(queue, distances);
        printf("\nmin node %d at queue index %d", queue[min], min);        
    }

    printf("\n");
}


int main() // u0, w1, v2, x3
{

    Edge e1 = {0, 1, 5, 2};
    Edge e2 = {0, 2, 10, 2};
    Edge e3 = {2, 1, 20, 4};
    Edge e4 = {1, 3, 10, 2};
    Edge e5 = {2, 3, 20, 1};

    Node u; u.id = 0;
    Node w; w.id = 1;
    Node v; v.id = 2;
    Node x; x.id = 3;

    u.in_edges.push_back(e1);
    u.in_edges.push_back(e2);
    v.in_edges.push_back(e3);
    w.in_edges.push_back(e4);
    v.in_edges.push_back(e5);
    
    Graph g; g.n = 4;
    g.nodes.push_back(u);
    g.nodes.push_back(w);
    g.nodes.push_back(v);
    g.nodes.push_back(x);

    g.print();
    printf("\n");
    g.print_out();

    printf("\nGrafo montado");
    printf("\nEscolher nó aleatório para começar\n");

    Calendar scheduler;
    int start = 0;
    int loop = 0;

    // adicionar evento de anuncio
    for (int i=0; i < g.nodes[start].in_edges.size(); i++)
    {
        Event e = {g.nodes[start].in_edges[i].source, g.nodes[start].in_edges[i].target, time(0)};
        scheduler.list.push_back(e);
    }
    
    while (scheduler.list.size() && loop < 10)
    {
        printf("\n============== %d =============== \n", loop);

        print_calendar(scheduler);

        // novos anuncios
        int current_node = scheduler.list[0].event.target;
        for (int i=0; i < g.nodes[current_node].in_edges.size(); i++)
        {
            Event e = {g.nodes[current_node].in_edges[i].source, g.nodes[current_node].in_edges[i].target, time(0)};
            scheduler.list.push_back(e);
        }
        print_calendar(scheduler);


        // validar distancia
        // ...

        //processa evento
        scheduler.list.erase(scheduler.list.begin());

        print_calendar(scheduler);
        loop++;
    }
    
    printf("\n");
    return 0;
}
