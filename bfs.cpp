#include <iostream>
#include <list>

using namespace std;

class Graph 
{

    public:
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
    Graph g(5);

    g.add_edge(1, 2);
    g.add_edge(4, 2);
    g.add_edge(1, 3);
    g.add_edge(4, 3);
    g.add_edge(1, 4);

    g.print();

    return 0;
}