
#ifndef OUTPUT_H_

#include "sm-astar.h"


void Paint(Map &map, const PNode &node, const PNode &startNode);

std::vector<std::string> RenderMap(Map &map, const PNode &node, const PNode &startNode);

#endif /* OUTPUT_H_ */
