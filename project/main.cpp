#include <stdlib.h>
#include <vector>
#include <iostream>
#include <ctime>  

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
    Edge event;
};

struct Calendar
{
    std::vector<Event> list;
};


std::vector<std::vector<std::pair<int,int>>> dist;
Graph g; 
Calendar scheduler;


void print_calendar(Calendar c)
{
    for (int i=0; i<c.list.size(); i++)
        std::cout << "\tevento " << i << ": ("
        << c.list[i].event.source << " > "
        << c.list[i].event.target << ")"
        << std::endl;
}


void print_distances(std::vector<std::vector<std::pair<int,int>>> dist)
{
    for (int i=0; i<dist.size(); i++)
    {
        printf("\n");
        for (int j=0; j<GRAPH_MAX_SIZE; j++)
            printf("\t(%4d; %4d)", dist[i][j].first, dist[i][j].second); //printf("%05d", i);
    }
}


void awake_node(int node_index)
{
    // add new events
    for (int i=0; i < g.nodes[node_index].out_edges.size(); i++)
    {
        Event e = {
            g.nodes[node_index].out_edges[i].source, 
            g.nodes[node_index].out_edges[i].target, 
            g.nodes[node_index].out_edges[i].width, 
            g.nodes[node_index].out_edges[i].length, 
        };
        scheduler.list.push_back(e);
    }

    // update distance to itself
    dist[node_index][node_index].first = 0;
    dist[node_index][node_index].second = 0;
}

// matrix with pairs width-length
void init()
{
    dist.resize(GRAPH_MAX_SIZE);
    for (int i=0; i<dist.size(); i++)
        for (int j=0; j<GRAPH_MAX_SIZE; j++)
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

    for (int i=0; i<distances.size(); i++) 
    {
        /*
        printf("\nrun %d ---- (%4d,%4d) vs (%4d,%4d)", 
            i, 
            distances[i].first, 
            distances[i].second, 
            current_width, 
            current_length);
        */

        // se distancia é menor ganha imediatamente
        if (current_length > distances[i].second || (current_length == distances[i].second && current_width < distances[i].first)) 
        {
            current_length = distances[i].second;
            current_width = distances[i].first;
            current_index = i;
        } /*
        if (current_length == distances[i].second && current_width < distances[i].first)
        {
            printf(" found at %d", i);
            current_length = distances[i].second;
            current_width = distances[i].first;
            current_index = i;       
        } */
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
        distances.push_back(EDGE_MAX_LENGHT);
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
        int min = 0; //minimum(queue, distances);
        printf("\nmin node %d at queue index %d", queue[min], min);        
    }

    printf("\n");
}


void bidirectional_dijkstra(Graph g, Node s, Node t)
{
    printf("\nBidirectional Dijkstra from %d to %d", s.id, t.id);
    printf("\n");

    std::vector<int> qs;
    std::vector<int> qt;

    std::vector<std::pair<int,int>> qs_distance;
    std::vector<std::pair<int,int>> qt_distance;
    std::vector<int> qs_parent;
    std::vector<int> qt_parent;

    bool found = false;
    int k = 0;

    // initialize everything
    for (int i=0; i<g.n; i++)
    {
        qs.push_back(g.nodes[i].id);
        qt.push_back(g.nodes[i].id);
        qs_parent.push_back(-1);
        qt_parent.push_back(-1);
        qs_distance.push_back(std::make_pair(EDGE_MAX_WIDTH, EDGE_MAX_LENGHT));
        qt_distance.push_back(std::make_pair(EDGE_MAX_WIDTH, EDGE_MAX_LENGHT));
    }

    // for itself, widths and enghts are zero
    qs_distance[s.id].first = 0; qs_distance[s.id].second = 0;
    qt_distance[t.id].first = 0; qt_distance[t.id].second = 0;


    while (qs.size())
    {

        printf("\n================================");
        printf("\n============== %d =============== \n", k);

        printf("\n\tQs");
        for (int i=0; i < qs.size(); i++)
            printf("\t%d", qs[i]);  

        printf("\n\tQt");
        for (int i=0; i < qt.size(); i++)
            printf("\t%d", qt[i]);  

        printf("\n\td-Qs");
        for (int i=0; i<g.n; i++)
            printf("\t(%4d,%4d)", qs_distance[i].first, qs_distance[i].second);
        printf("\n\td-Qt");
        for (int i=0; i<g.n; i++)
            printf("\t(%4d,%4d)", qt_distance[i].first, qt_distance[i].second);
    
        printf("\n\tp-Qs");
        for (int i=0; i<g.n; i++)
            printf("\t%d", qs_parent[i]);  
        printf("\n\tp-Qt");
        for (int i=0; i<g.n; i++)
            printf("\t%d", qt_parent[i]);  


        // trata da queue Qs
        int index = ws_minimum(qs, qs_distance); 
        int u = qs[index];
        printf("\n>>> qs: %d\n", u);

        // relaxar cada aresta uv
        for (int i=0; i<g.nodes[u].out_edges.size(); i++)
        {
            int v = g.nodes[u].out_edges[i].target;
            int uv_lenght = g.nodes[u].out_edges[i].length;
            int uv_width = g.nodes[u].out_edges[i].width;

            printf("\n\t target: %d (%d,%d)", 
                g.nodes[u].out_edges[i].target, 
                g.nodes[u].out_edges[i].width, 
                g.nodes[u].out_edges[i].length);
            
            printf("\n\t -> %d vs %d", qs_distance[u].first, uv_width);

            // se lenght mais curta do que está nas distancias, actualiza
            // fica com o menor valor para a width
            if (qs_distance[v].second > qs_distance[u].second + uv_lenght)
            {
                printf("*\n");
                qs_distance[v].second = qs_distance[u].second + uv_lenght;


                if (qs_parent[u] != -1)
                {
                    printf("\n\taqui %d vs %d ", qs_distance[u].first, uv_width);
                    qs_distance[v].first = qs_distance[u].first < uv_width ? qs_distance[u].first : uv_width;
                    printf("\tganha %d", qs_distance[v].first);
                }
                else
                    qs_distance[v].first = uv_width;
                
                qs_parent[v] = u;
            }
        }

        //remove nó da queue
        qs.erase(qs.begin() + index);


        // trata da queue Qt
        //u = ws_minimum(qt, qt_distance);
        //printf("\n>>>qt: %d", u);

        printf("\n");
        k++;
    }
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
    
    g.n = 4;
    g.nodes.push_back(u);
    g.nodes.push_back(w);
    g.nodes.push_back(v);
    g.nodes.push_back(x);

    g.print();
    printf("\n");
    g.print_out();

    int current_node = 0;
    int loop = 0;

    // adicionar evento de anuncio
    awake_node(current_node);
  
    while (scheduler.list.size() && loop < 0)
    {
        printf("\n================================");
        printf("\n============== %d =============== \n", loop);
        print_calendar(scheduler);

        print_distances(dist);

        // validar distancia
        printf("\n\n>>> analisar a distância de %d até %d", scheduler.list[0].event.source, scheduler.list[0].event.target);
        printf("\n\t obtemos (%d,%d)", scheduler.list[0].event.width, scheduler.list[0].event.length);

        // shortest-widest 
        // escolher a maior widht
        if (scheduler.list[0].event.width > dist[scheduler.list[0].event.source][scheduler.list[0].event.target].first)
        {
            printf("\n\t width é melhor, renova valor");
            dist[scheduler.list[0].event.source][scheduler.list[0].event.target].first = scheduler.list[0].event.width;
            dist[scheduler.list[0].event.source][scheduler.list[0].event.target].second = scheduler.list[0].event.length;
        }

        printf("\n");
        print_distances(dist);
        printf("\n");
        
        // novos anuncios
        current_node = scheduler.list[0].event.target;
        for (int i=0; i < g.nodes[current_node].out_edges.size(); i++)
        {
            Event e = {
                g.nodes[current_node].out_edges[i].source, 
                g.nodes[current_node].out_edges[i].target, 
                g.nodes[current_node].out_edges[i].width, 
                g.nodes[current_node].out_edges[i].length, 
            };
            scheduler.list.push_back(e);
        }

        //processa evento
        scheduler.list.erase(scheduler.list.begin());
        loop++;
    }
    
    printf("\nSource to destination");
    bidirectional_dijkstra(g, u, x);
    printf("\n");


    return 0;
}
