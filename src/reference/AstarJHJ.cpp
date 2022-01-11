#include "AstarJHJ.h"
#include <stdio.h>

bool AstarJHJ::MapSearchNode::IsSameState(MapSearchNode &rhs) {
  return pos == rhs.pos;
}

float AstarJHJ::MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal) {
  return map->distance(pos, nodeGoal.pos);
}

bool AstarJHJ::MapSearchNode::IsGoal(MapSearchNode &nodeGoal) {
  return pos == nodeGoal.pos;
}

bool AstarJHJ::MapSearchNode::GetSuccessors(
    AStarJHJSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node) {

  auto ns = map->neighbours(pos);

  MapSearchNode new_n;

  for (const auto &node : ns) {
    if (!parent_node || parent_node->pos != node) {
      new_n = MapSearchNode(map, node);
      astarsearch->AddSuccessor(new_n);
    }
  }

  return true;
}

float AstarJHJ::MapSearchNode::GetCost(MapSearchNode &successor) {
  return 1;
  // return map->cost(pos, successor.pos);
}

int AstarJHJ::MapSearchNode::Hash() { return pos; }

void AstarJHJ::MapSearchNode::PrintNodeInfo() { return; }

std::optional<std::vector<Node>> AstarJHJ::search(Node source, Node goal) {

  AStarJHJSearch<MapSearchNode> astarsearch;

  MapSearchNode nodeStart(&map, source);
  MapSearchNode nodeEnd(&map, goal);

  // Set Start and goal states
  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

  unsigned int SearchState;

  do {
    SearchState = astarsearch.SearchStep();
  } while (SearchState ==
           AStarJHJSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

  if (SearchState == AStarJHJSearch<MapSearchNode>::SEARCH_STATE_FAILED)
    return {};

  std::vector<Node> v;
  MapSearchNode *node = astarsearch.GetSolutionStart();

  node->PrintNodeInfo();
  for (;; node = astarsearch.GetSolutionNext()) {
    if (!node)
      break;
    v.emplace_back(node->pos);
  }

  // Once you're done with the solution you can free the nodes up
  // TODO for speed remove the following
  astarsearch.FreeSolutionNodes();
  astarsearch.EnsureMemoryFreed();

  return v;
}
