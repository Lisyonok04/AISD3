#pragma once

#include<iostream>
#include<list>
#include<vector>
#include<unordered_set>
#include<exception>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>

const int INF = std::numeric_limits<int>::max();

using namespace std;


template <typename V, typename Distance = double>
class Graph {
private:
	struct Edge {
		int _vert;
		Distance _distance;
	};
	struct Vertex {
		int _index;
		V _value;
		list<Edge> _edges;
	};
	vector<Vertex> _graph;

	Vertex& operator[](V& val) {
		for (auto& vertex : _graph) {
			if (vertex._value == val) {
				return vertex;
			}
		}
		throw invalid_argument("No such vertex");
	}

	void renum_after_delete(int start) {
		size_t count = 0;
		for (auto& i : _graph) {
			i._index = count;
			for (auto& edge : i._edges) {
				if (edge._vert >= start)
					edge._vert--;
			}
			count++;
		}
	}

	void dfs_h(int from, vector<int>& visited)const {
		visited[from] = 1;
		for (auto& i : _graph[from]._edges) {
			if (!visited[i._vert]) {
				dfs_h(i._vert, visited);
			}
		}
		cout << _graph[from]._value << " ";
	}
public:
	Graph() = default;
	~Graph() = default;

	void print() const {
		for (auto& vertex : _graph) {
			cout << vertex._value << " " << vertex._index;
			for (auto& edge : vertex._edges) {
				cout << "-->" << "|to " << _graph[edge._vert]._value << " - dist " << edge._distance << "|";
			}
			cout << endl;
		}
	}
	size_t size() const {
		return _graph.size();
	}
	bool has_vertex(const V& v) const {
		for (const auto& vertex : _graph)
			if (vertex._value == v)
				return true;
		return false;
	};
	void add_vertex(V v) {
		if (!has_vertex(v)) {
			int new_num = _graph.size();
			_graph.push_back({ new_num,  v });
		};
	};
	bool remove_vertex(V v) {
		try {
			auto index = (*this)[v]._index;
			for (auto& vertex : _graph) {
				vertex._edges.remove_if([&](Edge& x) { return _graph[x._vert]._value == v; });
			}
			_graph.erase(std::remove_if(_graph.begin(), _graph.end(), [v](Vertex& ver)
				{
					return ver._value == v;
				}
			), _graph.end());
			renum_after_delete(index);
			return true;
		}
		catch (invalid_argument& e) {
			return false;
		}
	};
	std::vector<Vertex> vertices() const {
		return _graph;
	};

	void add_edge(V from, V to, Distance d) {
		try {
			(*this)[from]._edges.push_back({ (*this)[to]._index, d });
		}
		catch (invalid_argument& e) {
			std::cerr << e.what() << std::endl;
		}
	};
	bool remove_edge(V from, V to) {
		if (has_vertex(from)) {
			(*this)[from]._edges.remove_if([&](Edge& x) { return _graph[x._vert]._value == to; });
			return true;
		}
		return false;
	};
	bool remove_edge(V from, V to, Distance val) {
		if (has_vertex(from)) {
			(*this)[from]._edges.remove_if([&](Edge& x) { return (_graph[x._vert]._value == to && x._distance == val); });
			return true;
		}
		return false;
	};
	bool has_edge(V from, V to, Distance val) {
		if (has_vertex(from)) {
			for (auto& edge : (*this)[from]._edges)
				if (_graph[edge._num]._value == to && edge._value == val)
					return true;
		}
		return false;
	};
	bool has_edge(V from, V to) {
		if (has_vertex(from)) {
			for (auto& edge : (*this)[from]._edge)
				if (_graph[edge._num]._val == to)
					return true;
		}
		return false;
	};
	list<Edge> edges(V from) const {
		if (has_vertex(from))
			return (*this)[from]._edge;
	}

	size_t order() const {
		return _graph.size();
	};
	size_t degree(V val) {
		return (*this)[val]._edge.size();
	}

	std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {

	};
	void bfs(V start_val) {
		auto start = (*this)[start_val]._index;
		queue<int> q;
		unordered_set<int> visited;

		q.push(start);
		visited.insert(start);

		while (!q.empty()) {
			int vertex = q.front();
			q.pop();

			cout << "Visiting vertex " << _graph[vertex]._value << endl;

			for (auto& neighbor : _graph[vertex]._edges) {
				if (visited.find(neighbor._vert) == visited.end()) {
					q.push(neighbor._vert);
					visited.insert(neighbor._vert);
				}
			}
		}
	}
	void dfs(V start_vertex) {
		if (has_vertex(start_vertex)) {
			vector<int> visited(size() + 1, 0);
			dfs_h((*this)[start_vertex]._index , visited);
		}
	}
	vector<double> Dijkstra(V _from, bool flag) {
		auto from = (*this)[_from]._index;
		vector<double> distance(size(), INF);
		distance[from] = 0;
		vector<vector<V>> paths(size());
		priority_queue<std::pair<int, int>, vector<std::pair<int, int>>, greater<>> queue;
		queue.push({ 0, from });

		while (!queue.empty()) {
			int u = queue.top().second;
			queue.pop();

			for (auto& edge : _graph[u]._edges) {
				int v = edge._vert;
				int weight = edge._distance;

				if (distance[v] > distance[u] + weight) {
					paths[v] = paths[u];
					paths[v].push_back(_graph[v]._value);
					distance[v] = distance[u] + weight;
					queue.push({ distance[v], v });
				}
			}
		}
		for (auto& elem : paths) {
			vector<V> new_vec;
			new_vec.push_back(_from);
			for (auto& i : elem)
				new_vec.push_back(i);
			elem = new_vec;
		}
		if (flag) {
			for (int i = 0; i < size(); i++) {
				std::cout << "From " << _from << " to " << _graph[i]._value << " is " << distance[i] << "path-> ";
				for (auto& elem : paths[i]) {
					cout << elem << " ";
				}
				cout << endl;
			}
			/*for (auto& vert : paths) {
				for (auto& elem : vert) {
					cout << elem << " ";
				}
				cout << endl;
			}*/
		}
		return distance;
	}
	V find_graph_center() {
		int center = -1;
		int max_dist = -1;
		for (auto& vertex : _graph) {
			vector<double> dist = Dijkstra(vertex._value, false);
			int max_d = *max_element(dist.begin(), dist.end());
			if (max_d < max_dist || max_dist == -1) {
				max_dist = max_d;
				center = vertex._index;
			}
		}
		return _graph[center]._value;
	}
};
