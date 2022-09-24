#include <iostream>
#include <list>

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


int main() 
{
    return 0;
}
