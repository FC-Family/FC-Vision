#include <limits>

#include "sop/history_node.h"

namespace fc_vision
{

HistoryNode::HistoryNode(int prefix_cost, int bound){
	this->prefix_cost = prefix_cost;
	this->lower_bound = bound;
}

HistoryNode::HistoryNode(int prefix_cost){
	this->prefix_cost = prefix_cost;
}

HistoryNode::HistoryNode(){
	prefix_cost = std::numeric_limits<int>::max();
	lower_bound = std::numeric_limits<int>::max();
}

} // namespace fc_vision
