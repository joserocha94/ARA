#include <iostream>
#include <list>
#include <vector>

using namespace std;

struct Graph
{
    int v;
    list<pair<int, int>> *adj;

    Graph(int n) 
    {
        adj = new list<pair<int, int>> [n];
        v = n;
    }

    void add_edge(int node_a, int node_b, int dist) 
    {
        adj[node_a].push_back(make_pair(node_b, dist));
        adj[node_b].push_back(make_pair(node_a, dist));
    }

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


int main()
{
    Graph g(4);

    g.add_edge(0, 1, 5);
    g.add_edge(0, 2, 5);
    g.add_edge(1, 3, 5);
    
    g.print();
    printf("\n");

    return 0;
}