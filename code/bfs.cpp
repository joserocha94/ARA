#include <iostream>
#include <list>
#include <vector>

using namespace std;

struct Graph
{
    int v;
    list<int> *adj;

    Graph(int n) 
    {
        adj = new list<int> [n];
        v = n;
    }

    void add_edge(int node_a, int node_b) 
    {
        adj[node_a].push_back(node_b);
    }

    void print()
    {
        for(int i=0; i<v; i++)
        {
            cout << i <<" --> ";
            for (auto it:adj[i])
                cout << it << " ";
            cout << endl;
        }    
    }
};

void bfs (Graph g, int start_node, int size)
{
    // mark all vertices as not visited
    vector<bool> visited;
    visited.resize(size);

    // created empty queue
    list<int> queue;

    // start node visited, enqueued
    visited[start_node] = true;
    queue.push_back(start_node);
    cout << endl;
    
    // loop to traverse the graph
    while (!queue.empty())
    {
        // Dequeue a vertex from queue and print it
        start_node = queue.front();
        cout << start_node << " ";
        queue.pop_front();
 
        // Get all adjacent vertices of the dequeued
        // vertex s. If a adjacent has not been visited,
        // then mark it visited and enqueue it
        for (auto adjecent: g.adj[start_node])
        {
            if (!visited[adjecent])
            {
                visited[adjecent] = true;
                queue.push_back(adjecent);
            }
        }
    }

    cout << endl;
}


int main()
{

    Graph g(4);

    g.add_edge(0, 1);
    g.add_edge(0, 2);
    g.add_edge(1, 2);
    g.add_edge(2, 0);
    g.add_edge(2, 3);
    g.add_edge(3, 3);

    g.print();

    bfs(g, 2, 4);


    return 0;
}