/*
Challenges:
constant time priority queue update/deletion
	done with a map that keeps track of what is in the priority queue

*/

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <climits>

using namespace std;

struct Key {
	int first;
	int second;

	Key() {}

	Key(int f, int s) {
		first = f;
		second = s;
	}

	Key(const Key &obj) {
		first = obj.first;
		second = obj.second;
	}

	bool operator<(const Key& other) const {
		return (first == other.first) ? second < other.second : first < other.first;
	}

	bool operator==(const Key& other) const {
        return first == other.first && second == other.second;
	}

    bool operator!=(const Key& other) const {
        return !(first == other.first && second == other.second);
    }
};

struct PQEntry {
	Key key;
	int id;

	PQEntry() {}

	PQEntry(Key k, int num) {
		key = Key(k);
		id = num;
	}

	bool operator==(const PQEntry& other) const {
		return key == other.key && id == other.id;
	}

	bool operator!=(const PQEntry& other) const {
		return !(key == other.key && id == other.id);
	}
};

struct Node {
	int id;
	int g;
	int rhs;
	vector<int> pred; 
	vector<int> succ; 

	Node() {}

	Node(int id, int g, int rhs, vector<int> pred, vector<int> succ) {
		this->g = g;
		this->rhs = rhs;
		this->id = id;
		this->succ = succ;
		this->pred = pred;
	}

	const void toString() const {
		cout << "id:" << id << " g:" << g << " rhs:" << rhs << " pred:";
		for (auto i : pred) {
			cout << i << ",";
		}
		cout << " succ:";
		for (auto i : succ) {
			cout << i << ",";
		}
		cout << endl;
	}
};

class KeySorter {
    public:
        bool operator() (PQEntry left, PQEntry right) {
            return ((left.key.first == right.key.first) ? right.key.second < left.key.second : right.key.first < left.key.first);
        }
};

class Graph {
	public:
    vector<vector<int>> currAdjMatrix;
	vector<vector<int>> actualAdjMatrix;
	unordered_map<int, pair<vector<int>, vector<int>>> adjLists; // maps the id of a node to a predecessor (.first) and successor (.second) list
    priority_queue<PQEntry, vector<PQEntry>, KeySorter> pq;
	unordered_map<int, PQEntry> pqMap;	// maps id to a 
	unordered_map<int, Node> nodeMap;
    vector<vector<int>> heuristic;
    int startId;
    int goalId;
    int lastStartId;
    int km;

    Graph(vector<vector<int>> cam, vector<vector<int>> aam, unordered_map<int, pair<vector<int>, vector<int>>> al, vector<vector<int>> h, int startID, int goalID) {
        currAdjMatrix = cam; // stores cost to travel to each node, -1 if impossible
		actualAdjMatrix = aam;
		adjLists = al;
        heuristic = h;
		startId = startID; 
		goalId = goalID;
		nodeMap[startId] = Node(startId, INT_MAX, INT_MAX, adjLists[startId].first, adjLists[startId].second);
		nodeMap[goalId] = Node(goalId, INT_MAX, 0, adjLists[goalId].first, adjLists[goalId].second);

		// procedure Initialize()
		km = 0;
		pq.push(PQEntry(Key(heuristic[startId][goalId],0), goalId));
		pqMap[goalId] = PQEntry(Key(heuristic[startId][goalId],0), goalId);
    }

	Key calculateKey(Node s) {
		Key key(min(s.g, s.rhs) + heuristic[startId][s.id] + km, min(s.g, s.rhs));
		return key;
	}

	// make sure node is already initialized when using this function
	void updateVertex(int nodeId) {
		if (nodeId != goalId) {
			nodeMap[nodeId].rhs = calculateRHS(nodeId);
		}
		if (nodeMap[nodeId].g != nodeMap[nodeId].rhs) {
			pqMap[nodeId] = PQEntry(calculateKey(nodeMap[nodeId]), nodeId);
			pq.push(pqMap[nodeId]);
		}
	}
	
	void computeShortestPath() {
		while(pq.top().key < calculateKey(nodeMap[startId]) || nodeMap[startId].rhs != nodeMap[startId].g)  {
			// continue if the top of the priority queue is either not in pqMap or not equal to what is in pqMap
			while (pqMap.count(pq.top().id) == 0 || pq.top() != pqMap[pq.top().id]) {
				pq.pop();
			}
			PQEntry pqTop = pq.top();
			pq.pop();
			Key newKey = calculateKey(nodeMap[pqTop.id]);
			if (pqTop.key < newKey) {
				pqMap[pqTop.id] = PQEntry(newKey, pqTop.id);
				pq.push(pqMap[pqTop.id]);
			}
			else if (nodeMap[pqTop.id].g > nodeMap[pqTop.id].rhs) {
				nodeMap[pqTop.id].g = nodeMap[pqTop.id].rhs;
				for (auto predId : nodeMap[pqTop.id].pred) {
					if (nodeMap.count(predId) == 0) {
						nodeMap[predId] = Node(predId, INT_MAX, INT_MAX, adjLists[predId].first, adjLists[predId].second);
					}
					updateVertex(predId);
				}
			}
			else {
				nodeMap[pqTop.id].g = INT_MAX;
				for (auto predId : nodeMap[pqTop.id].pred) {
					if (nodeMap.count(predId) == 0) {
						nodeMap[predId] = Node(predId, INT_MAX, INT_MAX, adjLists[predId].first, adjLists[predId].second);
					}
					updateVertex(predId);
				}
				updateVertex(pqTop.id);
			}
			while (pqMap.count(pq.top().id) == 0 || pq.top() != pqMap[pq.top().id]) {
				pq.pop();
			}
		}
	}

	void main() {
		cout << "Starting algorithm" << endl;
		lastStartId = startId;
		computeShortestPath();
		while (startId != goalId) {
			if (nodeMap[startId].g == INT_MAX) {
				cout << "No path possible" << endl;
			}
			int minDist = INT_MAX;
			int newStartId = startId;
			for (auto succId : nodeMap[startId].succ) {
				if (nodeMap.count(succId) == 0) {
					nodeMap[succId] = Node(succId, INT_MAX, INT_MAX, adjLists[succId].first, adjLists[succId].second);
				}
				int dist = safeAdd(currAdjMatrix[startId][succId], nodeMap[succId].g);
				if (minDist > dist) {
					minDist = dist;
					newStartId = succId;
				}
			}
			cout << "Moving from: " << startId << " to " << newStartId << endl;
			startId = newStartId;
			vector<pair<int,int>> changedEdges;				// assuming robot can only see successors of itself
			for (auto succId : nodeMap[startId].succ) {		
				if (currAdjMatrix[startId][succId] != actualAdjMatrix[startId][succId]) {
					if (nodeMap.count(succId) == 0) {
						nodeMap[succId] = Node(succId, INT_MAX, INT_MAX, adjLists[succId].first, adjLists[succId].second);
					}
					changedEdges.push_back(make_pair(startId,succId));
				}
			}
			if (changedEdges.size() != 0) {
				// cout << "hello" << endl;
				km += heuristic[lastStartId][startId];
				lastStartId = startId;
				for (auto edge : changedEdges) {
					currAdjMatrix[edge.first][edge.second] = actualAdjMatrix[edge.first][edge.second];
					updateVertex(edge.first);
				}
				computeShortestPath();
			}
		}
	}

	// can only use if nodeId has been initialized (if it is in nodeMap)
	int calculateRHS(int nodeId) {
		int minRhs = INT_MAX;
		for (auto succId : nodeMap[nodeId].succ) {
			if (nodeMap.count(succId) == 0) {
				nodeMap[succId] = Node(succId, INT_MAX, INT_MAX, adjLists[succId].first, adjLists[succId].second);
			}
			minRhs = min(minRhs, safeAdd(currAdjMatrix[nodeId][succId], nodeMap[succId].g));
		}
		return minRhs;
	}

	int safeAdd(int a, int b) {
		if (a == INT_MAX || b == INT_MAX) {
			return INT_MAX;
		}
		return a + b;
	}
};

int main() {
	// sample test case
	vector<vector<int>> adjMatrix = {{0,1,-1,-1,-1},
									 {1,0,1,1,-1},
									 {-1,1,0,1,1},
									 {-1,1,1,0,10},
									 {-1,-1,1,10,0}};
	vector<vector<int>> changedAdjMatrix = {{0,1,-1,-1,-1},
											{1,0,INT_MAX,1,-1},
											{-1,INT_MAX,0,INT_MAX,INT_MAX},
									 		{-1,1,INT_MAX,0,10},
									 		{-1,-1,INT_MAX,10,0}};
	vector<vector<int>> heuris = 	{{0,1,2,2,3},
									 {1,0,1,1,2},
									 {2,1,0,1,1},
									 {2,1,1,0,1},
									 {3,2,1,1,0}};
	unordered_map<int, pair<vector<int>, vector<int>>> adjlists;
	for (int i = 0; i < 5; i ++) {
		vector<int> pred;
		vector<int> succ;
		for (int j = 0; j < 5; j++) {
			if (adjMatrix[i][j] >= 1) {
				pred.push_back(j);
				succ.push_back(j);
			} 
		}
		adjlists[i] = make_pair(pred, succ);
	}
	int stid = 0;
	int goalid = 4;

	Graph graph(adjMatrix, changedAdjMatrix, adjlists, heuris, stid, goalid);
	graph.main();
}