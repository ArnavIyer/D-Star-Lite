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
};

class KeySorter {
    public:
        bool operator() (PQEntry left, PQEntry right) {
            return ((left.key.first == right.key.first) ? right.key.second > left.key.second : right.key.first < left.key.first);
        }
};

class Graph {
    vector<vector<int>> initialAdjMatrix;
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

    Graph(vector<vector<int>> iam, vector<vector<int>> aam, unordered_map<int, pair<vector<int>, vector<int>>> al, vector<vector<int>> h, int startID, int goalID) {
        initialAdjMatrix = iam; // stores cost to travel to each node, -1 if impossible
		actualAdjMatrix = aam;
		adjLists = al;
        heuristic = h;
		startId =startID; //Node(startId, INT_MAX, INT_MAX, adjLists[startId].first, adjLists[startId].second);
		goalId =goalID;// Node(goalId, INT_MAX, 0, adjLists[startId].first, adjLists[startId].second);
		nodeMap[startId] = Node(startId, INT_MAX, INT_MAX, adjLists[startId].first, adjLists[startId].second);
		nodeMap[goalId] = Node(goalId, INT_MAX, 0, adjLists[goalId].first, adjLists[goalId].second);

		// procedure Initialize()
		km = 0;
		pq.push(PQEntry(Key(heuristic[startId][goalId],0), goalId));
    }

	Key calculateKey(Node s) {
		Key key(min(s.g, s.rhs) + heuristic[startId][s.id] + km, min(s.g, s.rhs));
		return key;
	}

	// make sure node is already initialized when using this function
	void updateVertex(int nodeId) {
		if (nodeMap[nodeId].g != nodeMap[nodeId].rhs) {
			pqMap[nodeId] = PQEntry(calculateKey(nodeMap[nodeId]),nodeId);
			pq.push(pqMap[nodeId]);
		} else if (nodeMap.count(nodeId) == 1 && nodeMap[nodeId].g != nodeMap[nodeId].rhs) {
			pqMap.erase(nodeId);
		}33
	}
	
	void computeShortestPath() {
		while(pq.top().key < calculateKey(nodeMap[startId]) || nodeMap[startId].rhs > nodeMap[startId].g)  {
			// continue if the top of the priority queue is either not in pqMap or not equal to what is in pqMap
			PQEntry pqTop = pq.top();
			if (pqMap.count(pqTop.id) == 0 || pqMap[pqTop.id] != pqTop)
				continue;
			
			Key kNew = calculateKey(nodeMap[pqTop.id]);
			if (pqTop.key < kNew) {
				pqMap[pqTop.id] = PQEntry(kNew,pqTop.id);
				pq.push(pqMap[pqTop.id]);
			} 
			else if (nodeMap[pqTop.id].g > nodeMap[pqTop.id].rhs) {
				nodeMap[pqTop.id].g = nodeMap[pqTop.id].rhs;
				pqMap.erase(pqTop.id);
				pq.pop();
				for (auto predId : nodeMap[pqTop.id].pred) {
					if (nodeMap.count(predId) == 0) {
						nodeMap[predId] = Node(predId, INT_MAX, INT_MAX, adjLists[predId].first, adjLists[predId].second);
					}
					if (predId != goalId) {
						nodeMap[predId].rhs = min(nodeMap[predId].rhs, safeAdd(initialAdjMatrix[predId][pqTop.id], nodeMap[pqTop.id].g));
					}
					updateVertex(predId);
				}
			}
			else {
				int gOld = nodeMap[pqTop.id].g;
				nodeMap[pqTop.id].g = INT_MAX;
				for (auto predId : nodeMap[pqTop.id].pred) {
					if (nodeMap.count(predId) == 0) {
						nodeMap[predId] = Node(predId, INT_MAX, INT_MAX, adjLists[predId].first, adjLists[predId].second);
					}
					if (nodeMap[predId].rhs == initialAdjMatrix[predId][pqTop.id] + gOld) {
						if (predId != goalId) {
							nodeMap[predId].rhs = calculateRHS(predId);
						}
					}
					updateVertex(predId);
				}
				// also have to do it for pqTop.id
				if (nodeMap[pqTop.id].rhs == gOld) {
					if (pqTop.id != goalId) {
						nodeMap[pqTop.id].rhs = calculateRHS(pqTop.id);
					}
				}
				updateVertex(pqTop.id);
			}
		}
	}

	void main() {
		lastStartId = startId;
		computeShortestPath();
		while (startId != goalId) {
			if (nodeMap[startId].rhs == INT_MAX) {
				cout << "No path possible" << endl;
				return;
			}
			int minDist = INT_MAX;
			for (auto succId : nodeMap[startId].succ) {
				if (nodeMap.count(succId) == 0) {
					nodeMap[succId] = Node(succId, INT_MAX, INT_MAX, adjLists[succId].first, adjLists[succId].second);
				}
				int dist = safeAdd(initialAdjMatrix[startId][succId], nodeMap[succId].g);
				if (minDist > dist) {
					minDist = dist;
					startId = succId;
				}
			}
			vector<pair<int,int>> changedEdges;
			
		// scan graph for changed edges, add to changed edges list
		// if changed edges list is not empty,
		}
	}

	// can only use if nodeId has been initialized (if it is present in the map)
	int calculateRHS(int nodeId) {
		int minRhs = 0;
		for (auto succId : nodeMap[nodeId].succ) {
			if (nodeMap.count(succId) == 0) {
				nodeMap[succId] = Node(succId, INT_MAX, INT_MAX, adjLists[succId].first, adjLists[succId].second);
			}
			minRhs = min(minRhs, safeAdd(initialAdjMatrix[nodeId][succId], nodeMap[succId].g));
		}
		return minRhs;
	}

	int safeAdd(int a, int b) {
		if (a == INT_MAX || b == INT_MAX) {
			return INT_MAX;
		}
		return a + b;
	}
    
    // instead of updating the key of a node in a priority queue, you have to
    // check everytime you are popping something if you are popping the right version of the node

};