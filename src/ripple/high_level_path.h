#pragma once

#include <vector>

#include "CLionProjects/parallel-ripple-search/src/graph/map.h"

Path<Node> create_high_level_path(Map &map, Node source, Node goal);
