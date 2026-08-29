#include <vector>

#include "sop/edge.h"
#include "sop/solver_state.h"

using std::vector;

namespace fc_vision
{

SolverState::SolverState(Edge next_edge, std::vector<Edge> path, int cost, int bound){
	this->next_edge = next_edge;
	this->path = path;
	this->cost = cost;
	this->lower_bound = bound;
}

SolverState::SolverState(const SolverState& state){
	this->next_edge = state.next_edge;
	this->path = state.path;
	this->cost = state.cost;
	this->lower_bound = state.lower_bound;
}

} // namespace fc_vision