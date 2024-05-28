#pragma once

#include<iostream>
#include<list>
#include<vector>
#include<exception>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>

const int INF = std::numeric_limits<int>::max();

using namespace std;


//template <typename Vertex, typename Distance = double>
//struct Vertexes {
//	struct Edge {
//		int _vert;
//		Distance _distance;
//	};
//	int _index;
//	Vertex _value;
//	list<Edge> _edges;
//	Vertexes(int index, Vertex value) : _index(index), _value(value) {};
//};
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
		//Vertex(int index, Vertex value) : _index(index), _value(value) {};
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
	std::vector<Vertex> walk(const Vertex& start_vertex)const {

	};
};
