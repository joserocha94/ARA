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

// debug 
void print_list(list<int> queue)
{
    cout << endl;  
    printf("[Queue]:");
    for (auto it = queue.begin(); it != queue.end(); ++it)
        cout << "\t" << *it;
    cout << endl;
}

// function to find the minimum distance for every node
// stored. this is being done several times
// so we should implement a priority_queue in the project
int min(list<int> queue, vector<int> distances)
{
    int distance = 9999;
    int index = 0;

    for (list<int>::iterator it = queue.begin(); it != queue.end(); ++it)
    {
        if (distances[*it] < distance)
        {    
            distance = distances[*it];
            index = *it; 
        }
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

    // create a queue to keep all the vertices 
    // keeping track of the distances during execution
    // to do: change this to a priority queue to avoid search
    list<int> queue;
    for(int i=0; i<g.v; i++) 
        queue.push_back(i);
    
    while (queue.size())
    {    
        // find minimum element on the queue  
        int minimum = min(queue, dist);        

        // remove it
        for (list<int>::iterator it = queue.begin(); it != queue.end(); ++it)
            if (*it == minimum) 
                it = queue.erase(it);
            
        // foreach uv from that connects the choosen node
        // we update the distance (relaxation process)
        for (auto it:g.adj[minimum])
            if (dist[it.first] > dist[minimum] + it.second)
                dist[it.first] = dist[minimum] + it.second; 
        
        cout << endl;
        print_list(queue);     
    }
}

int main()
{
    Graph g(4);

    g.add_edge(0, 1, 8);
    g.add_edge(0, 2, 5);
    g.add_edge(1, 3, 2);
    
    g.print();
    cout << endl;

    dijkstra(g, 0);
    cout << endl;

    return 0;
}
