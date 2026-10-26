#include <iostream> //cin,cout
#include <fstream>  //files
#include <queue>
#include <vector>
#include <stack>
#include <map>
#include <iomanip>
#include <cmath>    // sqrt, pow
#include <cstdlib>  // exit
#include <string>
using namespace std;


struct nodeType // structure of each node in the tree
{
    int state;    // the index of the vertex
    int parent;   // the state of the parent node
    double costG; // g(n), path cost
    double costH; // h(n), estimated cost by heuristic from present to goal
    // overload the < operator so we can use it for the priority queue
    friend bool operator<(nodeType x, nodeType y)
    {
        return x.costG + x.costH > y.costG + y.costH;
    };
};

//To make the code compatible on Mac and windows
void pauseProgram()
{
    cout << "\nPress Enter to exit...";
    cin.get();
}


struct navigationGraph
{
    int size;      // number of vertices
    double **adjM; // adjacency matrix
    vector<double> xloc;
    vector<double> yloc;
};
int ACount = 0;
int UCcount = 0;
bool readGraphFromFile(string filename, navigationGraph &g1); // given a file, populate the graph
vector<nodeType> Expand(nodeType curr, navigationGraph g1);
// given a current node curr and the graph, return the nodes you can reach from curr.
vector<nodeType> Solution(int currentId, int startId, map<int, nodeType> exploredNodes);
// Helper function: returns the solution path given the map of explored nodes by the algorithm
vector<nodeType> uniform_cost_search_Nav(navigationGraph g1, int start, int goal); // uniform cost search between two nodes, returns a path
vector<nodeType> astar_search_Nav(navigationGraph g1, int start, int goal);        // A* search between two nodes, returns a path
//////////////////////////////////////////////////////////////
int main()
{
    string filename;

    string files[10] = {
    "data/P2x2m.txt",
    "data/P2x4s.txt",
    "data/P3x3.txt",
    "data/P3x4s.txt",
    "data/P3x8s.txt",
    "data/P4x6.txt",
    "data/P3x10.txt",
    "data/P5x10.txt",
    "data/P10x10.txt",
    "data/P15x10.txt",
};
    cout << setw(10) << "Filename" << setw(10) << "Cost" << setw(20) << "UC cost size" << setw(20) << "A* cost size" << "\tSolution Path (UC/A*)" << endl;
    for (int i = 0; i < 10; i++)
    {
        navigationGraph g1;
        int su, sa;
        vector<int> pu, pa;
        filename = files[i];
        cout << setw(15) << filename.substr(filename.find('/') + 1);   // prints P2x2m.txt
        if (!readGraphFromFile(filename, g1))                          // still opens data/P2x2m.txt
            exit(1);
        int start = 0, goal = g1.size - 1;
        su = 0;
        sa = 0;
        // getting the structure of both cost and the nodes
        vector<nodeType> pathu = uniform_cost_search_Nav(g1, start, goal);
        vector<nodeType> patha = astar_search_Nav(g1, start, goal);
        cout << fixed << setprecision(1);
        cout << setw(10) << pathu[0].costG;
        cout << setw(20) << UCcount;
        cout << setw(20) << ACount;
        for (auto x : pathu)
            pu.insert(pu.begin(), x.state);
        for (auto x : patha)
            pa.insert(pa.begin(), x.state);
        cout << setw(5) << " ";
        for (auto x : pu)
            cout << setw(4) << x;
        cout << endl;
        cout << setw(30) << " ";
        for (auto x : pa)
            cout << setw(4) << x;
        cout << endl;
    }
    pauseProgram();
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////
bool readGraphFromFile(string filename, navigationGraph &g1)
{
    ifstream gfile(filename);
    if (!gfile)
    {
        cout << "Error opening file ..." << endl;
        return false;
    }
    int m, n; // #of vertices, #of edges,
    gfile >> m >> n;
    g1.size = m; // set the size of the graph
    // allocate memory for the adjacacy matrix
    g1.adjM = new double *[m];
    for (int i = 0; i < m; i++)
        g1.adjM[i] = new double[m]{0};
    int id;                     // vertex id
    double x, y;                // x-y coordinates
    for (int i = 0; i < m; i++) // read the vertex information
    {
        gfile >> id >> x >> y;
        g1.xloc.push_back(x);
        g1.yloc.push_back(y);
    }
    int a, b; // read the edges and fill out the adjacency matrix
    for (int i = 0; i < n; i++)
    {
        gfile >> a >> b;
        g1.adjM[a][b] = 1;
        g1.adjM[b][a] = 1;
    }
    gfile.close();
    return true;
}
vector<nodeType> Expand(nodeType curr, navigationGraph g1)
{
    vector<nodeType> children;
    int id = curr.state; // the id of current node
    nodeType newnode;
    int numChildren = 0;
    newnode.parent = id;
    for (int i = 0; i < g1.size; i++)
    {
        if (g1.adjM[id][i] > 0) // there is a connection between vertices id and i
        {
            double d = sqrt(pow(g1.xloc[id] - g1.xloc[i], 2) + pow(g1.yloc[id] - g1.yloc[i], 2));
            newnode.costG = curr.costG + d; // update the cost g(n)=cost of parent+ cost of transition
            newnode.state = i;
            newnode.costH = 0; // the heuristic cost is assumed 0 (the informed searches will update this themselves)
            children.push_back(newnode);
        }
    }
    return children;
}
vector<nodeType> Solution(int currentId, int startId, map<int, nodeType> exploredNodes)
{
    vector<nodeType> path;
    nodeType curr = exploredNodes[currentId];
    path.push_back(curr);
    while (curr.parent >= 0)
    {
        curr = exploredNodes[curr.parent];
        path.push_back(curr);
    }
    return path;
}
// Put here your function implementations
vector<nodeType> uniform_cost_search_Nav(navigationGraph g1, int start, int goal)
{
    // intializing first state
    nodeType firstNode;
    firstNode.state = 0;
    firstNode.parent = -1;
    firstNode.costG = 0;
    firstNode.costH = 0;
    // global count varaible
    UCcount = 0;
    // storing the final goal state
    double lastX = g1.xloc[goal];
    double lastY = g1.yloc[goal];
    // building frontier queue
    priority_queue<nodeType> frontier;
    frontier.push(firstNode);
    // building the reached map
    map<int, nodeType> reachedNodes;
    reachedNodes[0] = firstNode;
    // final vector for nodes
    vector<nodeType> returnNodes;
    // vectors for frontier children
    vector<nodeType> frontierChildren;
    // the node to be used in the loop
    nodeType node;
    while (!frontier.empty())
    {
        node = frontier.top();
        frontier.pop();
        if (node.state == goal)
        {
            returnNodes = Solution(node.state, firstNode.state, reachedNodes);
            UCcount = reachedNodes.size();
            return returnNodes;
        }
        frontierChildren = Expand(node, g1);
        for (nodeType child : frontierChildren)
        {
            if ((reachedNodes.find(child.state) == reachedNodes.end()) || (child.costG < reachedNodes[child.state].costG))
            {
                // cout << endl << "Cost " <<child.state <<" is " << child.costG << endl;
                reachedNodes[child.state] = child;
                // cout << "The reachedNodes is " <<child.state <<"    " <<  reachedNodes[child.state].costG;
                frontier.push(child);
            }
        }
    }
    // failure
    UCcount = reachedNodes.size();
    return returnNodes;
}
vector<nodeType> astar_search_Nav(navigationGraph g1, int start, int goal)
{
    // intializing first state
    nodeType firstNode;
    firstNode.state = 0;
    firstNode.parent = -1;
    firstNode.costG = 0;
    firstNode.costH = 0;
    // for the count global varaible
    ACount = 0;
    // storing the final goal state
    double lastX = g1.xloc[goal];
    double lastY = g1.yloc[goal];
    // building frontier queue
    priority_queue<nodeType> frontier;
    frontier.push(firstNode);
    // building the reached map
    map<int, nodeType> reachedNodes;
    reachedNodes[0] = firstNode;
    // final vector for nodes
    vector<nodeType> returnNodes;
    // vectors for frontier children
    vector<nodeType> frontierChildren;
    // the node to be used in the loop
    nodeType node;
    while (!frontier.empty())
    {
        node = frontier.top();
        frontier.pop();
        if (node.state == goal)
        {
            returnNodes = Solution(node.state, firstNode.state, reachedNodes);
            ACount = reachedNodes.size();
            return returnNodes;
        }
        frontierChildren = Expand(node, g1);
        for (nodeType child : frontierChildren)
        {
            // setting the heuristic cost
            child.costH = sqrt(pow(lastX - g1.xloc[child.state], 2) + pow(lastY - g1.yloc[child.state], 2));
            if ((reachedNodes.find(child.state) == reachedNodes.end()) || (child.costH + child.costG < reachedNodes[child.state].costG + reachedNodes[child.state].costH))
            {
                reachedNodes[child.state] = child;
                frontier.push(child);
            }
        }
    }
    // failure
    ACount = reachedNodes.size();
    return returnNodes;
}