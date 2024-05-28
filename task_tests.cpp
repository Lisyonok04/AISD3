#include "task.cpp"
#include <gtest/gtest.h>
#include <stdexcept>
#include <iostream>
TEST(GraphTests, VertexCheck) {
	Graph<int> g;
	g.size();
	g.add_vertex(5);
	g.add_vertex(7);
	g.add_vertex(9);
	g.add_edge(5, 7, 10);
	g.add_edge(5, 9, 20);
	g.add_edge(7, 9, 30);
	g.print();
	cout << endl;
	g.remove_vertex(7);
	g.remove_edge(5, 9, 20);
	g.print();
};

TEST(GraphTests_2, VertexCheck) {
	Graph<int> g;
	g.add_vertex(1);
	g.add_vertex(2);
	g.add_vertex(3);
	g.add_vertex(4);
	g.add_vertex(5);
	g.add_edge(2,1,10);
	g.add_edge(2,3,20);
	g.add_edge(3,4,30);
	g.add_edge(3,5,40);
	g.print();
	g.Dijkstra(2, true);
	cout << g.find_graph_center();
};