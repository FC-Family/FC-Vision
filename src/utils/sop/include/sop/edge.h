#ifndef EDGE_H
#define EDGE_H

namespace fc_vision {

class Edge {
	public:
		Edge(int source, int dest, int weight);
		Edge();
		int source;
		int dest;
		int weight;
};

bool operator<(const Edge& first, const Edge& second);

} // namespace fc_vision

#endif
