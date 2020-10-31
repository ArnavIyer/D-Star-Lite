#ifndef IMPORTS
#define IMPORTS
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#endif

using namespace std;

class GridToGraph {
    public:
        vector<vector<bool>> grid;
        vector<vector<bool>> actualGrid;

        GridToGraph() {}

        GridToGraph(vector<vector<bool>> grid, vector<vector<bool>> actualGrid) {
            this->grid = grid;
            this->actualGrid = actualGrid;
        }

        int id(int i, int j) {
            return i * grid.size() + j;
        }

        auto getData() {
            const int dx[4] = {1,0,-1,0};
            const int dy[4] = {0,1,0,-1};  
            int numNodes = grid.size()*grid[0].size();

            vector<vector<int>> adjMatrix(numNodes, vector<int> (numNodes, 0));
            vector<vector<int>> actualAdjMatrix(numNodes, vector<int> (numNodes, 0));
            unordered_map<int, pair<vector<int>, vector<int>>> adjLists; // pred is .first succ is .second
            vector<vector<int>> heuristics(numNodes, vector<int> (numNodes, 0));

            vector<int> temp;
            adjLists[0] = make_pair(temp, temp);
            for (int i = 0; i < grid.size(); i++) {
                for (int j = 0; j < grid[0].size(); j++) {
                    int thisId = id(i,j);
                    for (int k = 0; k < 4; k++) {
                        if (i+dx[k] >= 0 && i+dx[k] < grid.size() && j+dy[k] >= 0 && j+dy[k] < grid.size()) {
                            int otherId = id(i+dx[k], j+dy[k]);
                            if (grid[i+dx[k]][j+dy[k]]) {
                                adjMatrix[thisId][otherId] = 1;
                                adjMatrix[thisId][otherId] = 1;
                                if (adjLists.count(otherId) == 0) {
                                    adjLists[otherId] = make_pair(temp, temp);
                                }
                                adjLists[otherId].first.push_back(thisId);
                                adjLists[thisId].second.push_back(otherId);
                            } else {
                                adjMatrix[thisId][otherId] = INT_MAX;
                                adjMatrix[thisId][otherId] = INT_MAX;
                            }
                        }
                    }
                    if (j > i) {
                        for (int n = 0; n < grid.size(); n++) {
                            for (int m = 0; m < grid[0].size(); m++) {
                                heuristics[id(i,j)][id(n,m)] = hypot(i-n, j-m);
                                heuristics[id(n,m)][id(i,j)] = hypot(i-n, j-m);
                            }
                        }
                    }
                }
            }
            for (int i = 0; i < grid.size(); i++) {
                for (int j = 0; j < grid[0].size(); j++) {
                    int thisId = id(i,j);
                    for (int k = 0; k < 4; k++) {
                        if (i+dx[k] >= 0 && i+dx[k] < grid.size() && j+dy[k] >= 0 && j+dy[k] < grid.size()) {
                            int otherId = id(i+dx[k], j+dy[k]);
                            if (actualGrid[i+dx[k]][j+dy[k]]) {
                                actualAdjMatrix[thisId][otherId] = 1;
                                actualAdjMatrix[thisId][otherId] = 1;
                            } else {
                                actualAdjMatrix[thisId][otherId] = INT_MAX;
                                actualAdjMatrix[thisId][otherId] = INT_MAX;
                            }
                        }
                    }
                }
            }
            return make_tuple(adjMatrix, actualAdjMatrix, adjLists, heuristics);
        }
};