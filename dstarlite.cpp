#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

struct NodePQ {
    int first;
    int second;
    int node;

    NodePQ(int f, int s, int n) {
        first = f;
        second = s;
        node = n;
    }
};

class dStarLitePQSorter {
    public:
        bool operator() (NodePQ left, NodePQ right) {
            return ((left.first == right.first) ? right.second > left.second : right.first < left.first);
        }
};

class Graph {
    vector<vector<int>> adjList;
    priority_queue<NodePQ, vector<NodePQ>, dStarLitePQSorter> pq;
    vector<int> g;
    vector<vector<int>> heuristic;
    vector<int> rhs;
    int start;
    int end;
    int lastStart;
    int km;

    Graph(vector<vector<int>> al, int st, int e, vector<vector<int>> h) {
        adjList = al;
        heuristic = h;
        for (int i = 0; i < adjList.size(); i++) {
            g.push_back(INT_MAX);
            rhs.push_back(INT_MAX);
        }
        rhs[0] = 0;
        start = st;
        end = e;
        lastStart = st;
        km = 0;
    }
};

int main() {

}