#include <iostream>
#include <list>
#include <vector>
#include <bits/stdc++.h>

using namespace std;


// Graph struct to discretize the problem;
// representation based on an adjacency list
// to do: is it the best option?
struct Graph
{
    int v;
    list<pair<int, int>> *adj;

    // graph-sized constructor
    // to do: change to vector struct
    Graph(int n) 
    {
        adj = new list<pair<int, int>> [n];
        v = n;
    }

    // add edge between two different nodes
    // to do: add validation?
    void add_edge(int node_a, int node_b, int dist) 
    {
        adj[node_a].push_back(make_pair(node_b, dist));
        adj[node_b].push_back(make_pair(node_a, dist));
    }

    // print the graph as an adjacency list
    void print()
    {
        for(int i=0; i<v; i++) 
        {
            cout << i <<" --> ";
            for (auto it:adj[i])
                cout << "(" << it.first << ";" << it.second << ") ";
            cout << endl;
        }    
    }
};


void print_list(list<pair<int,int>> queue)
{
    cout << endl;
    cout << endl;
    
    list<pair<int,int>>::iterator it;
    for (it = queue.begin(); it != queue.end(); ++it)
        cout << "(" << (*it).first << ";" << (*it).second << ") ";
    
    cout << endl;
}


// function to find the minimum distance for every node
// stored. this is being done several times
// so we should implement a priority_queue in the project
// to do: that curr over there makes me cry
int min(vector<int> queue)
{
    int distance = 9999;
    int index = 0;

    for (int i=0; i < queue.size(); i++)
        if (queue[i] < distance)
        {
            distance = queue[i];
            index = i;
        }

    return index;
}


// implementation of Dijkstra's algorithm to find minimum distances
// rule: if d[v] > d[u] + l(u,v) 
//          d[v] := d[u] + l(u,v);
void dijkstra(Graph g, int source)
{

    // initialize all the distances with a max default value
    // the source node has distance equals to zero
    vector<int> dist(g.v, 9999);
    dist[source] = 0;


    for(int i=0; i<g.v; i++)
        printf("\t %d", dist[i]);


    // create a queue to keep all the vertices 
    // keeping track of the distances during execution
    // to do: change this to a priority queue to avoid search
    list<pair<int,int>> queue;
    for(int i=0; i<g.v; i++) 
        for (auto it:g.adj[i])
            queue.push_back(it);
    print_list(queue);

    printf("\nmin found at position %d", min(dist));
   
    // to do
}


int main()
{
    Graph g(4);

    g.add_edge(0, 1, 5);
    g.add_edge(0, 2, 5);
    g.add_edge(1, 3, 5);
    
    g.print();
    printf("\n");

    dijkstra(g, 0);
    printf("\n");

    return 0;
}
