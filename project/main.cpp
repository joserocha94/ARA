#include <stdlib.h>
#include <vector>
#include <iostream>

#define GRAPH_MAX_SIZE 4
#define EDGE_MAX_DISTANCE 999

// structs will be definided in a header file
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
    std::vector<Edge> edges;
};

struct Graph
{
    int n;
    std::vector<Node> nodes;

    void print()
    {
        for (int i=0; i<nodes.size(); i++)
            for (int j=0; j <nodes[i].edges.size(); j++)
                std::cout << "[" 
                << nodes[i].id  << "] (" 
                << nodes[i].edges[j].source << "," 
                << nodes[i].edges[j].target << ")" 
                << std::endl;
    }
};

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

// heap struct...
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
        printf("\nelements %d", queue.size()); std::fflush(stdout);
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

    u.edges.push_back(e1);
    u.edges.push_back(e2);
    v.edges.push_back(e3);
    v.edges.push_back(e4);
    w.edges.push_back(e5);
    
    Graph g; g.n = 4;
    g.nodes.push_back(u);
    g.nodes.push_back(w);
    g.nodes.push_back(v);
    g.nodes.push_back(x);

    g.print();
    dijkstra(g, v);

    return 0;
}
