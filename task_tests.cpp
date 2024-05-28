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