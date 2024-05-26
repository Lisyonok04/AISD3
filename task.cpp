#pragma once

#include<iostream>
#include<list>
#include<vector>
#include<exception>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>

using namespace std;

template <typename Vertex, typename Distance = double>
class Graph {
private:
	struct Edge {
		int _vert;
		Distance _value;
	};
	struct Vertex {
		int _num;
		Vertex _val;
		list<Edge> _edge;
	};