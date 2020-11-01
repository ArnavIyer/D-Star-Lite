#ifndef IMPORTS
#define IMPORTS
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#endif
#include <cstdlib>

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
            return i * grid[0].size() + j;
        }

        auto getData() {
            const int dx[4] = {1,0,-1,0};
            const int dy[4] = {0,1,0,-1};  
            int numNodes = grid.size()*grid[0].size();

            vector<vector<int>> adjMatrix(numNodes, vector<int> (numNodes, 0));
            vector<vector<int>> actualAdjMatrix(numNodes, vector<int> (numNodes, 0));
            unordered_map<int, pair<vector<int>, vector<int>>> adjLists; // pred is .first succ is .second
            vector<vector<int>> heuristics(numNodes, vector<int> (numNodes, 0));
            vector<bool> isObstacle(numNodes);
            vector<bool> actualIsObstacle(numNodes);

            vector<int> temp;
            adjLists[0] = make_pair(temp, temp);
            for (int i = 0; i < grid.size(); i++) {
                for (int j = 0; j < grid[0].size(); j++) {
                    isObstacle[id(i,j)] = grid[i][j] == 0;
                    actualIsObstacle[id(i,j)] = actualGrid[i][j] == 0;
                    int thisId = id(i,j);
                    for (int k = 0; k < 4; k++) {
                        if (i+dx[k] >= 0 && i+dx[k] < grid.size() && j+dy[k] >= 0 && j+dy[k] < grid[0].size()) {
                            int otherId = id(i+dx[k], j+dy[k]);
                            if (adjLists.count(otherId) == 0) {
                                adjLists[otherId] = make_pair(temp, temp);
                            }
                            adjLists[otherId].first.push_back(thisId);
                            adjLists[thisId].second.push_back(otherId);
                            if (grid[i+dx[k]][j+dy[k]] && grid[i][j]) {
                                adjMatrix[thisId][otherId] = 1;
                                adjMatrix[otherId][thisId] = 1;
                            } else {
                                adjMatrix[thisId][otherId] = INT_MAX;
                                adjMatrix[otherId][thisId] = INT_MAX;
                            }
                        }
                    }
                }
            }
            for (int i = 0; i < grid.size(); i++) {
                for (int j = 0; j < grid[0].size(); j++) {
                    int thisId = id(i,j);
                    for (int k = 0; k < 4; k++) {
                        if (i+dx[k] >= 0 && i+dx[k] < grid.size() && j+dy[k] >= 0 && j+dy[k] < grid[0].size()) {
                            int otherId = id(i+dx[k], j+dy[k]);
                            if (actualGrid[i+dx[k]][j+dy[k]] && actualGrid[i][j]) {
                                actualAdjMatrix[thisId][otherId] = 1;
                                actualAdjMatrix[otherId][thisId] = 1;
                            } else {
                                actualAdjMatrix[thisId][otherId] = INT_MAX;
                                actualAdjMatrix[otherId][thisId] = INT_MAX;
                            }
                        }
                    }
                }
            }
            abs(0.5);
            for (int i = 0; i < numNodes; i++) {
                for (int j = 0; j < numNodes; j++) {
                    int numCols = grid[0].size();
                    heuristics[i][j] = std::abs(i / numCols - j / numCols) + std::abs(i % numCols - j % numCols);
                    heuristics[j][i] = std::abs(i / numCols - j / numCols) + std::abs(i % numCols - j % numCols);
                }
            }
            return make_tuple(adjMatrix, actualAdjMatrix, adjLists, heuristics, isObstacle, actualIsObstacle);
        }
};