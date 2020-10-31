#ifndef IMPORTS
#define IMPORTS
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#endif

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

struct GraphNode {
	int id;
	int g;
	int rhs;
	vector<int> pred; 
	vector<int> succ; 

	GraphNode() {}

	GraphNode(int id, int g, int rhs, vector<int> pred, vector<int> succ) {
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