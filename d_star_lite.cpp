#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include "graph_structs.cpp"
#include "grid_to_graph.cpp"

using namespace std;
using namespace cv;

static void editAdjList(int event, int x, int y, int flags, void* userdata);

class Graph {
    public:
        const Scalar RED = Scalar(0, 0, 255);
        const Scalar GREEN = Scalar(0, 255, 0);
        const Scalar GRAY = Scalar(128, 128, 128);
        const Scalar BLACK = Scalar(0, 0, 0);
        const Scalar WHITE = Scalar(255, 255, 255);
        int pixelsPerSquare = 50;

        vector<vector<int>> currAdjMatrix;
        vector<vector<int>> actualAdjMatrix;
        unordered_map<int, pair<vector<int>, vector<int>>> adjLists; // maps the id of a node to a predecessor (.first) and successor (.second) list
        priority_queue<PQEntry, vector<PQEntry>, KeySorter> pq;
        unordered_map<int, PQEntry> pqMap;    // maps id to a
        unordered_map<int, GraphNode> nodeMap;
        vector<vector<int>> heuristic;
        int startId;
        int goalId;
        int lastStartId;
        int km;

        Mat img;
        bool useVisualizer;
        int numCols;
        vector<bool> isObstacle;
        vector<bool> actualIsObstacle;
        vector<int> propogatedTo;

        Graph() {}

        Graph(vector<vector<int>> cam, vector<vector<int>> aam,
              unordered_map<int, pair<vector<int>, vector<int>>> al,
              vector<vector<int>> h, int startID, int goalID, bool uv,
              int nc = 0, vector<bool> isObstacle = vector<bool>(),
              vector<bool> actualIsObstacle = vector<bool>()) {
            currAdjMatrix = cam; // stores cost to travel to each node
            actualAdjMatrix = aam;
            adjLists = al;
            heuristic = h;
            startId = startID;
            goalId = goalID;
            nodeMap[startId] = GraphNode(startId, INT_MAX, INT_MAX,
                                         adjLists[startId].first,
                                         adjLists[startId].second);
            nodeMap[goalId] = GraphNode(goalId, INT_MAX, 0,
                                        adjLists[goalId].first,
                                        adjLists[goalId].second);
            numCols = nc;
            if (numCols > 0) {
                img = Mat::zeros(pixelsPerSquare * currAdjMatrix.size() / numCols,
                                 pixelsPerSquare * numCols, CV_8UC3);
                this->isObstacle = isObstacle;
                this->actualIsObstacle = actualIsObstacle;
            }
            useVisualizer = uv;
            pixelsPerSquare = 50;

            // procedure Initialize()
            km = 0;
            pq.push(PQEntry(Key(heuristic[startId][goalId],0), goalId));
            pqMap[goalId] = PQEntry(Key(heuristic[startId][goalId],0), goalId);
        }

        Key calculateKey(GraphNode s) {
            if (s.g == INT_MAX && s.rhs == INT_MAX) {
                return Key(INT_MAX, INT_MAX);
            }
            Key key(min(s.g, s.rhs) + heuristic[startId][s.id] + km, min(s.g,
                                                                         s.rhs));
            return key;
        }

        // make sure node is already initialized when using this function
        void updateVertex(int nodeId) {
            if (nodeId != goalId) {
                nodeMap[nodeId].rhs = calculateRHS(nodeId);
            }
            if (pqMap.count(nodeId) == 1) {
                pqMap.erase(nodeId);
            }
            if (nodeMap[nodeId].g != nodeMap[nodeId].rhs) {
                pqMap[nodeId] = PQEntry(calculateKey(nodeMap[nodeId]), nodeId);
                pq.push(pqMap[nodeId]);
            }
        }

        void computeShortestPath() {
            propogatedTo.clear();
            while(pq.top().key < calculateKey(nodeMap[startId]) ||
                  nodeMap[startId].rhs != nodeMap[startId].g)  {
                // continue if the top of the priority queue is either not in pqMap or not equal to what is in pqMap
                while (pq.size() > 0 && (pqMap.count(pq.top().id) == 0 ||
                       pq.top() != pqMap[pq.top().id])) {
                    pq.pop();
                }
                PQEntry pqTop = pq.top();
                if (useVisualizer) {
                    propogatedTo.push_back(pqTop.id);
                }
                pq.pop();
                pqMap.erase(pqTop.id);
                Key newKey = calculateKey(nodeMap[pqTop.id]);
                if (pqTop.key < newKey) {
                    pqMap[pqTop.id] = PQEntry(newKey, pqTop.id);
                    pq.push(pqMap[pqTop.id]);
                }
                else if (nodeMap[pqTop.id].g > nodeMap[pqTop.id].rhs) {
                    nodeMap[pqTop.id].g = nodeMap[pqTop.id].rhs;
                    for (auto predId : nodeMap[pqTop.id].pred) {
                        if (nodeMap.count(predId) == 0) {
                            nodeMap[predId] = GraphNode(predId, INT_MAX, INT_MAX,
                                                        adjLists[predId].first,
                                                        adjLists[predId].second);
                        }
                        updateVertex(predId);
                    }
                }
                else {
                    nodeMap[pqTop.id].g = INT_MAX;
                    for (auto predId : nodeMap[pqTop.id].pred) {
                        if (nodeMap.count(predId) == 0) {
                            nodeMap[predId] = GraphNode(predId, INT_MAX, INT_MAX,
                                                        adjLists[predId].first,
                                                        adjLists[predId].second);
                        }
                        updateVertex(predId);
                    }
                    updateVertex(pqTop.id);
                }
                while (pq.size() > 0 && (pqMap.count(pq.top().id) == 0 ||
                       pq.top() != pqMap[pq.top().id])) {
                    pq.pop();
                }
            }
        }

        void main() {
            cout << "Starting algorithm" << endl;
            lastStartId = startId;
            computeShortestPath();
            if (useVisualizer) {
                drawGraph();
                namedWindow("Visualizer", 1);
                imshow("Visualizer", img);
                setMouseCallback("Visualizer", editAdjList, NULL);
                waitKey(0);
            }
            while (startId != goalId) {
                if (nodeMap[startId].g == INT_MAX) {
                    cout << "No path possible" << endl;
                }
                int minDist = INT_MAX;
                int newStartId = startId;
                for (auto succId : nodeMap[startId].succ) {
                    if (nodeMap.count(succId) == 0) {
                        nodeMap[succId] = GraphNode(succId, INT_MAX, INT_MAX,
                                                    adjLists[succId].first,
                                                    adjLists[succId].second);
                    }
                    int dist = safeAdd(currAdjMatrix[startId][succId],
                                       nodeMap[succId].g);
                    if (minDist > dist) {
                        minDist = dist;
                        newStartId = succId;
                    }
                }
                cout << "Moving from: " << startId << " to " << newStartId << endl;
                startId = newStartId;
                vector<pair<int,int>> changedEdges;                // assuming robot can only see successors of itself
                for (auto succId : nodeMap[startId].succ) {
                    if (nodeMap.count(succId) == 0) {
                        nodeMap[succId] = GraphNode(succId, INT_MAX, INT_MAX,
                                                    adjLists[succId].first,
                                                    adjLists[succId].second);
                    }
                    if (currAdjMatrix[startId][succId] != actualAdjMatrix[startId][succId]) {
                        changedEdges.push_back(make_pair(startId, succId));
                    }
                    if (currAdjMatrix[startId][succId] != actualAdjMatrix[startId][succId]) {
                        changedEdges.push_back(make_pair(succId, startId));
                    }
                }
                if (changedEdges.size() != 0) {
                    km += heuristic[lastStartId][startId];
                    lastStartId = startId;
                    if (useVisualizer) {
                        for (auto edge : changedEdges) {
                            isObstacle[edge.second] = actualIsObstacle[edge.second];
                        }
                    }
                    for (auto edge : changedEdges) {
                        if (useVisualizer && edge.second != startId) {
                            if (isObstacle[edge.second]) {
                                for (auto neighbor : nodeMap[edge.second].succ) {
                                    currAdjMatrix[edge.second][neighbor] = INT_MAX;
                                    currAdjMatrix[neighbor][edge.second] = INT_MAX;
                                }
                            } else {
                                for (auto neighbor : nodeMap[edge.second].succ) {
                                    if (!isObstacle[neighbor]) {
                                        currAdjMatrix[edge.second][neighbor] = 1;
                                        currAdjMatrix[neighbor][edge.second] = 1;
                                    }
                                }
                            }
                        }
                        currAdjMatrix[edge.first][edge.second] = actualAdjMatrix[edge.first][edge.second];
                        updateVertex(edge.first);
                    }
                    computeShortestPath();
                }
                if (useVisualizer) {
                    drawGraph();
                    namedWindow("Visualizer", 1);
                    imshow("Visualizer", img);
                    waitKey(0);
                }
            }
            waitKey(0);
        }

        // can only use if nodeId has been initialized (if it is in nodeMap)
        int calculateRHS(int nodeId) {
            int minRhs = INT_MAX;
            for (auto succId : nodeMap[nodeId].succ) {
                if (nodeMap.count(succId) == 0) {
                    nodeMap[succId] = GraphNode(succId, INT_MAX, INT_MAX,
                                                adjLists[succId].first,
                                                adjLists[succId].second);
                }
                minRhs = min(minRhs, safeAdd(currAdjMatrix[nodeId][succId],
                                             nodeMap[succId].g));
            }
            return minRhs;
        }

        int safeAdd(int a, int b) {
            if (a == INT_MAX || b == INT_MAX) {
                return INT_MAX;
            }
            return a + b;
        }

        Rect getRectFromId(int nodeId) {
            return Rect(nodeId % numCols * pixelsPerSquare,
                        nodeId / numCols * pixelsPerSquare,
                        pixelsPerSquare, pixelsPerSquare);
        }

        Point getCenterPointFromId(int nodeId) {
            return Point(nodeId % numCols * pixelsPerSquare + pixelsPerSquare / 2,
                         nodeId / numCols * pixelsPerSquare + pixelsPerSquare / 2);
        }

        void drawGraph() {
            // reset img
            img.setTo(WHITE);

            for (const auto& kv : nodeMap) {
                if (kv.second.g == INT_MAX && kv.second.rhs == INT_MAX) {
                    rectangle(img, getRectFromId(kv.first), Scalar(119,219,244),
                              FILLED); // magenta
                }
                else if (kv.second.rhs == INT_MAX) {
                    rectangle(img, getRectFromId(kv.first), Scalar(230,119,244),
                              FILLED); // yellow
                }
                else if (kv.second.g == INT_MAX) {
                    rectangle(img, getRectFromId(kv.first), Scalar(244,147,119),
                              FILLED); // blue
                }
                else {
                    rectangle(img, getRectFromId(kv.first), Scalar(139,239,160),
                              FILLED); // green
                }
            }

            // draw goal
            rectangle(img, getRectFromId(goalId), RED, FILLED);

            // draw robot
            rectangle(img, getRectFromId(startId), GREEN, FILLED);

            // loop through actualAdjMatrix and color obstacles gray
            for (int i = 0; i < actualAdjMatrix.size(); i++) {
                if (actualIsObstacle[i])
                    rectangle(img, getRectFromId(i), GRAY, FILLED);
            }

            // loop through currAdjMatrix and color obstacles black
            for (int i = 0; i < currAdjMatrix.size(); i++) {
                if (isObstacle[i])
                    rectangle(img, getRectFromId(i), BLACK, FILLED);
            }

            // all nodes expanded (removed from priority queue) in the previous iteration - red outline
            for (auto updated : propogatedTo) {
                rectangle(img, getRectFromId(updated), RED);
            }

            // locally inconsistent - blue outline
            for (const auto& kv : nodeMap) {
                if (kv.second.g != kv.second.rhs) {
                    rectangle(img, getRectFromId(kv.first), Scalar(255,0,0));
                }
            }

            // draw current planned path
            int currNode = startId;
            while (currNode != goalId) {
                int minId = nodeMap[currNode].succ[0];
                int min = nodeMap[nodeMap[currNode].succ[0]].g;
                for (auto succ : nodeMap[currNode].succ) {
                    if (nodeMap[succ].g < min) {
                        min = nodeMap[succ].g;
                        minId = succ;
                    }
                }
                line(img, getCenterPointFromId(currNode),
                     getCenterPointFromId(minId), RED, 2);
                currNode = minId;
            }
        }

        int coordToId(int x, int y) {
            return y / pixelsPerSquare * numCols + x / pixelsPerSquare;
        }
};

vector<vector<bool>> grid = {{1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                             {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

vector<vector<bool>> actualGrid = { {1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1},
                                    {1,0,1,0,0,1,0,1,1,1,1,1,0,0,1,0,0,1,1,0,0,1,0,1,0,1,1,0},
                                    {1,1,1,1,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,1},
                                    {0,0,0,1,1,0,1,1,1,1,1,1,1,0,0,1,1,0,1,1,0,1,0,1,0,0,1,1},
                                    {1,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,1,0,1,1,0,1,1,1,0,1},
                                    {1,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1},
                                    {1,1,1,1,0,1,1,0,1,0,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,1,0,1},
                                    {1,1,0,0,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,0,0,1},
                                    {1,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,1,1,1,1},
                                    {1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0}};


GridToGraph gridToGraph(grid, actualGrid);
auto data = gridToGraph.getData();

Graph graphVisualizer(get<0>(data),get<1>(data),get<2>(data),get<3>(data), gridToGraph.id(0, 0),
                        gridToGraph.id(0, 26), true, grid[0].size(), get<4>(data), get<5>(data));

static void editAdjList(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        int clickId = graphVisualizer.coordToId(x, y);
        for (auto succId : graphVisualizer.adjLists[clickId].first) {
            graphVisualizer.actualAdjMatrix[succId][clickId] = INT_MAX;
            graphVisualizer.actualAdjMatrix[clickId][succId] = INT_MAX;
            graphVisualizer.actualIsObstacle[clickId] = true;
        }
    }
    else if (event == EVENT_RBUTTONDOWN) {
        cout << graphVisualizer.coordToId(x, y) << endl;
    }
}
int main() {
    // sample test case with visualizer (to use visualizer, you need to use gridToGraph)
//    vector<vector<bool>> grid = {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                 {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};
//
//    vector<vector<bool>> actualGrid = { {1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
//                                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};
//
//
//    GridToGraph gridToGraph(grid, actualGrid);
//    auto data = gridToGraph.getData();
//
//    Graph graphVisualizer(get<0>(data),get<1>(data),get<2>(data),get<3>(data), gridToGraph.id(0, 0),
//                          gridToGraph.id(0, 26), true, grid[0].size(), get<4>(data), get<5>(data));
    graphVisualizer.main();

    // sample test case without visualizer (you cannot use grid to graph)
    // vector<vector<int>> adjMatrix = {{0,1,-1,-1,-1},
    //                                  {1,0,1,1,-1},
    //                                  {-1,1,0,1,1},
    //                                  {-1,1,1,0,10},
    //                                  {-1,-1,1,10,0}};
    // vector<vector<int>> changedAdjMatrix = {{0,1,-1,-1,-1},
    //                                         {1,0,INT_MAX,1,-1},
    //                                         {-1,INT_MAX,0,INT_MAX,INT_MAX},
    //                                         {-1,1,INT_MAX,0,10},
    //                                         {-1,-1,INT_MAX,10,0}};
    // vector<vector<int>> heuris =     {{0,1,2,2,3},
    //                                  {1,0,1,1,2},
    //                                  {2,1,0,1,1},
    //                                  {2,1,1,0,1},
    //                                  {3,2,1,1,0}};
    // unordered_map<int, pair<vector<int>, vector<int>>> adjlists;
    // for (int i = 0; i < 5; i ++) {
    //     vector<int> pred;
    //     vector<int> succ;
    //     for (int j = 0; j < 5; j++) {
    //         if (adjMatrix[i][j] >= 1) {
    //             pred.push_back(j);
    //             succ.push_back(j);
    //         }
    //     }
    //     adjlists[i] = make_pair(pred, succ);
    // }
    // int stid = 0;
    // int goalid = 4;

    // Graph graph(adjMatrix, changedAdjMatrix, adjlists, heuris, stid, goalid, false);
    // graph.main();
}

// compile: g++ d_star_lite.cpp -o a `pkg-config --cflags --libs opencv`