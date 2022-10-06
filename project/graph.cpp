#include <stdlib.h>
#include <vector>
#include <iostream>

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
