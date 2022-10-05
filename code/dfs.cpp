#include <iostream>
#include <list>
#include <vector>

using namespace std;

struct Graph
{
    int v;
    vector<int> *adj;

    Graph(int n) 
    {
        adj = new vector<int> [n];
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
        cout << endl;   
    }
};

void dfs (Graph g, int node, vector<bool> visited)
{
    //  node visited, printed
    visited[node] = true;
    cout << node << " ";
    
    // find sucessors
    vector<int>::iterator new_node;

    for (new_node = g.adj[node].begin(); new_node != g.adj[node].end(); ++new_node)
    {
        if (!visited[*new_node])
            dfs(g, *new_node, visited);
    }
}


int main()
{
    Graph g(4);

    vector<bool> visited;
    visited.resize(g.v);

    g.add_edge(0, 1);
    g.add_edge(0, 2);
    g.add_edge(1, 2);
    g.add_edge(2, 0);
    g.add_edge(2, 3);
    g.add_edge(3, 3);

    g.print();
    dfs(g, 2, visited);

    cout << endl;
    return 0;
}