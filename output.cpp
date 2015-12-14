#include "output.h"

#include <iostream>



void Paint(Map &map, const PNode &curNode, const PNode &startNode)
{
	std::vector<std::string> pic = RenderMap(map, curNode, startNode);

	std::cout << " ";
	for (int x = 0; x < map.Width(); ++x)
		std::cout << x % 10;
	std::cout << std::endl;

	std::cout << std::string(map.Width()+2, '#') << std::endl;

	int c = 0;
	for (std::string &l : pic)
		std::cout << '#' << l << "# " << c++ << std::endl;

	std::cout << std::string(map.Width()+2, '#') << std::endl;

	std::cout << " ";
	for (int x = 0; x < map.Width(); ++x)
		std::cout << x % 10;
	std::cout << std::endl;

}


std::vector<std::string> RenderMap(Map &map, const PNode &node, const PNode &startNode)
{
	std::vector<std::string> rv;
	for (int y = 0; y < map.Height(); ++y) {
		std::string line = "";
		for (int x = 0; x < map.Width(); ++x) {
			if (map.Passable(x, y))
				line += " ";
			else
				line += "#";
		}
		rv.push_back(line);
	}

	PNode n = startNode;
	NodeSet q;
	do {
		for (PNode c : n->Children()) {
			q.insert(c);
		}
		n.reset();
		if (!q.empty()) {
			n = *q.begin();
			q.erase(q.begin());
			rv[map.Y(n->Offset())][map.X(n->Offset())] = '.';
		}
	} while (n);

	n = node;
	while (n) {
		for (PNode c : n->Children()) {
			if (rv[map.Y(c->Offset())][map.X(c->Offset())] != 'o') {
				if (map.Y(n->Offset()) == map.Y(c->Offset()))
					rv[map.Y(c->Offset())][map.X(c->Offset())] = '-';
				else
					rv[map.Y(c->Offset())][map.X(c->Offset())] = '|';
			}
		}
		if (n->Parent()) {
			rv[map.Y(n->Offset())][map.X(n->Offset())] = 'o';
			n = n->Parent();
		}
		else {
			rv[map.Y(n->Offset())][map.X(n->Offset())] = 'S';
			break;
		}
	}
	rv[map.Y(map.targetOffset_)][map.X(map.targetOffset_)] = 'T';
	return rv;
}

