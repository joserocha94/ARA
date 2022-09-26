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


void euler(Graph g, int start_node)
{
    int new_header, new_tail, aux; // aux does nothing
    std::vector<int> head;
    std::vector<int> tail;

    head.insert(head.begin(), start_node);

    //for (int i=0; i<head.size(); i++)
    //    printf("\n%d", head[i]);

    while (head.size())
    {
        while (g.adj[head.front()].size())
        {
            printf("\n[working on %d, remove edge %d --- %d, add %d to head]", 
                head.front(), 
                head.front(), 
                g.adj[head.front()].back(), 
                g.adj[head.front()].back()); 
            fflush(stdout);
            new_header = g.adj[head.front()].back();
            
            aux = head.front();
            g.adj[head.front()].pop_back();
            head.insert(head.begin(), new_header);

            // find related ege
            int counter = 0;
            for (auto it:g.adj[new_header])
            {
                if (it == aux)
                {
                    aux = it;
                    break;
                }
                counter++;
            }

            // remove it
            printf("\nremove from index %d", counter);


            cout << endl;
            cout << endl;
            g.print();

            break;
        }    
break;
        // While top node v of head is isolated and head is not empty 
        while (!g.adj[head.front()].size() && head.size())
        {
            printf("\n=================================");
            printf("\nhead: %d", head.front());
            printf("\ntop-node-neighbours: %d,  head-size: %d", g.adj[head.front()].size(), head.size());

            new_tail = head.front();
            tail.insert(tail.begin(), new_tail);
            head.erase(head.begin());

            printf("\nhead: %d", head.front());
            printf("\ntop-node-neighbours: %d,  head-size: %d", g.adj[head.front()].size(), head.size());
            printf("\n=================================");
            
            cout << endl;
            cout << endl;
        }
    }
    
    cout << endl;
    for (int i=0; i<head.size(); i++)
        printf("\t%d", head[i]);

    cout << endl;
    for (int i=0; i<tail.size(); i++)
        printf("\t%d", tail[i]);

    cout << endl;
}

int main()
{
    Graph g(4);

    g.add_edge(0, 1);
    g.add_edge(0, 2);
    g.add_edge(1, 0);
    g.add_edge(1, 2);
        g.add_edge(2, 1);
    g.add_edge(2, 0);
    g.add_edge(2, 3);
    g.add_edge(3, 2);

    g.print();
    euler(g, 1);

    return 0;
}