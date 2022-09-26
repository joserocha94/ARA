#include <iostream>
#include <list>
#include <vector>

using namespace std;

// add_egde should add the edge in the both vertices
// create and delete edge method doing the opposite
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

// euler algorithm finds an euler circuit in graph;
// each edge is traveled only once
// add degree validation!
void euler(Graph g, int start_node)
{
    int new_header, new_tail, aux;
    std::vector<int> head;
    std::vector<int> tail;

    head.insert(head.begin(), start_node);

    while (head.size())
    {
        // while top-head is non-isolated node
        // get edge uv, remove it, add v to the head
        while (g.adj[head.front()].size())
        {
            new_header = g.adj[head.front()].back();
            
            aux = head.front();
            g.adj[head.front()].pop_back();
            head.insert(head.begin(), new_header);

            g.adj[new_header].remove(aux);
        }    

        // while top-head isn't isolated or empty
        // remove node from head, add node from tail
        while (!g.adj[head.front()].size() && head.size())
        {
            new_tail = head.front();
            tail.insert(tail.begin(), new_tail);
            head.erase(head.begin());          
        }
    }

    // print the founded euler circuit
    cout << endl;
    for (int i=0; i<tail.size(); i++)
        printf("\t%d", tail[i]);

    cout << endl;
}

int main()
{
    Graph g(6);

    g.add_edge(0, 1);
    g.add_edge(0, 5);

    g.add_edge(1, 0);
    g.add_edge(1, 2);
    g.add_edge(1, 3);
    g.add_edge(1, 5);

    g.add_edge(2, 1);
    g.add_edge(2, 3);

    g.add_edge(3, 1);
    g.add_edge(3, 2);    
    g.add_edge(3, 4);
    g.add_edge(3, 5);

    g.add_edge(4, 3);
    g.add_edge(4, 5);

    g.add_edge(5, 0);
    g.add_edge(5, 1);
    g.add_edge(5, 3);
    g.add_edge(5, 4);

    g.print();
    euler(g, 1);

    return 0;
}
