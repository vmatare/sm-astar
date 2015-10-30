#include <cmath>
#include <limits>
#include <cstring>
#include <memory>
#include <exception>
#include <iostream>
#include <numeric>

#include "sma-star.h"

const char Map::searchX_[] = {1,  0, -1,  0};
const char Map::searchY_[] = {0, -1,  0,  1};
WeakNodeSet Node::allNodes_;
int Node::count_ = 0;


template<class T> size_t HashByOffset<T>::operator() (const T &node) const {
	return node->Offset();
}
///*
template<> size_t HashByOffset<const std::weak_ptr<Node> &>::operator() (const std::weak_ptr<Node> &node) const {
	return node.lock()->Offset();
}//*/
template size_t HashByOffset<PNode>::operator() (const PNode &node) const;
//template size_t HashByOffset<WeakPNode>::operator() (const WeakPNode &node) const;


template<class T> bool EqualByOffset<T>::operator() (
		const T &node1,
		const T &node2) const {
	return node1->Offset() == node2->Offset();
}
///*
template<> bool EqualByOffset<std::weak_ptr<Node>>::operator() (
		const std::weak_ptr<Node> &node1,
		const std::weak_ptr<Node> &node2) const {
	return node1.lock()->Offset() == node2.lock()->Offset();
}//*/
template bool EqualByOffset<const PNode &>::operator() (const PNode &node1, const PNode &node2) const;
//template bool EqualByOffset<const WeakPNode &>::operator() (const WeakPNode &node1, const WeakPNode &node2) const;



Node::Node(const Map &map, const int x, const int y, const PNode &parent, NodeQueue &queue) :
		offset_(map.Offset(x, y)), gCost_(parent ? parent->gCost_ + 1 : 0),
		hCost_(map.HCost(offset_)), serial_(count_++)
{
	PNode sh_this(this);
	std::pair<WeakNodeSet::iterator, bool> ins = allNodes_.insert(sh_this);
	if (ins.second) {
		NodeQueue::iterator this_it = queue.find(sh_this);
		if (parent && this_it == queue.end()) {
			this->parents_.insert(parent);
			parent->children_.insert(sh_this);
		}
		if (this_it == queue.end())
			queue.insert(std::move(sh_this));
	}
}


Node::~Node()
{
	allNodes_.erase(PNode(this));
}


void Node::Erase(NodeQueue &queue, bool eraseFromQ)
{
	PNode sh_this(*allNodes_.find(PNode(this)));
	if (!children_.empty()) {
		while (!children_.empty()) {
			const PNode &c = *children_.begin();
			c->Erase(queue);
		}
	}
	while (!parents_.empty()) {
		const PNode &p = *parents_.begin();
		p->children_.erase(sh_this);
	}
	if (eraseFromQ)
		queue.erase(sh_this);
}


void Node::SimplifyPath(const Map &map, NodeQueue &queue)
{
	Path path(std::move(FullPath()));
	if (path.size() < 3) return;

	Path::reverse_iterator path_it = path.rbegin();
	while (path_it != path.rend()) {
		PNode n1 = *path_it;
		PNode n2 = *(++path_it);
		PNode n3;
		int x = map.X(n2->offset_) - map.X(n1->offset_);
		int y = map.Y(n2->offset_) - map.Y(n1->offset_);

		while(true) {
			n3 = *(++path_it);
			int next_x = map.X(n3->offset_) - map.X(n2->offset_);
			int next_y = map.Y(n3->offset_) - map.Y(n2->offset_);
			if (next_x == x && next_y == y && queue.find(n2) == queue.end())
				n2->Erase(queue, false);
			else
				break;
			n2 = n3;
		}
	}
}


std::vector<PNode> Node::FullPath()
{
	PNode n(this);
	std::vector<PNode>rv;
	while (n->parents_.size() > 0) {
		rv.push_back(n);
		n = n->BestParent();
	}
	rv.push_back(n);
	return std::move(rv);
}


inline const unsigned int Node::Count() { return allNodes_.size(); }
inline NodeSet &Node::Children()        { return children_; }
inline NodeSet &Node::Parents()         { return parents_; }
inline const int Node::Offset() const   { return offset_; }
inline const float Node::FCost() const  { return hCost_ + float(gCost_); }
inline const int &Node::GCost() const   { return gCost_; }
inline const float &Node::HCost() const { return hCost_; }
inline const int Node::Serial() const   { return serial_; }

inline void Node::SetHCost(float hCost, NodeQueue &queue)
{
	bool in_q = queue.erase(PNode(this)) > 0;
	this->hCost_ = hCost;
	if (in_q)
		queue.insert(std::move(PNode(this)));
}


PNode Node::BestParent() const
{
	PNode rv(nullptr);
	int min_g = std::numeric_limits<int>::max();
	for (const PNode &n : parents_) {
		if (n->gCost_ < min_g) {
			min_g = n->gCost_;
			rv = std::move(n);
		}
	}
	return std::move(rv);
}


Map::Map(const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
		const int targetX, const int targetY) :
		pMap_(pMap),
		nMapWidth_(nMapWidth),
		nMapHeight_(nMapHeight),
		targetOffset_(_CART_TO_OFFSET(targetX, targetY, nMapWidth))
{}

inline unsigned int Map::X(const int offset) const
{ return offset % nMapWidth_; }

inline unsigned int Map::Y(const int offset) const
{ return offset / nMapWidth_; }

inline int Map::Offset(const int x, const int y) const
{ return _CART_TO_OFFSET(x, y, nMapWidth_); }

inline int Map::Width() const
{ return nMapWidth_; }

inline int Map::Height() const
{ return nMapHeight_; }


void Map::Expand(const PNode &n, NodeQueue &queue) const
{
	for (int i = 0; i <= 3; i++) {
		int x = X(n->Offset()) + searchX_[i];
		int y = Y(n->Offset()) + searchY_[i];
		if (Passable(x, y) && Offset(x, y) != n->Offset()) {
			new Node(*this, x, y, n, queue);
		}
	}
}


inline bool Map::Passable(const int x, const int y) const
{ return x >= 0 && y >= 0 && x < nMapWidth_ && y < nMapHeight_ && *(pMap_ + Offset(x, y)); }


float Map::HCost(const int from) const
{
	float a = ((long int)X(targetOffset_)) - X(from);
	float b = ((long int)Y(targetOffset_)) - Y(from);
	return std::sqrt(a*a + b*b);
}






/**
    @param nStartX and @param nStartY are the 0-based coordinates of the start position.
    @param nTargetX and @param nTargetY are the 0-based coordinates of the target position.
    @param pMap describes a grid of width nMapWidth and height nMapHeight. The grid
    is given in row-major order, each row is given in order of increasing x-coordinate,
    and the rows are given in order of increasing y-coordinate. Traversable locations of
    the grid are indicated by 1, and impassable locations are indicated by 0.
    Locations are considered to be adjacent horizontally and vertically but not diagonally.
    @param pOutBuffer is where you should store the positions visited in the found path, excluding
    the starting position but including the final position. Entries in pOutBuffer are
    indices into pMap. If there is more than one shortest path from Start to Target, any
    one of them will be accepted.
    @param nOutBufferSize is the maximum number of entries that can be written to pOutBuffer.

	@return The length of the shortest path between Start and Target, or −1
	if no such path exists.
	If the shortest path is longer than nOutBufferSize, the calling function might either
	give up or choose to call FindPath again with a larger output buffer.

	Constraints

    1 ≤ nMapWidth,nMapHeight,
    0 ≤ nStartX, nTargetX < nMapWidth,
    0 ≤ nStartY, nTargetY < nMapHeight,
    Both Start and Target are empty locations,
    nOutBufferSize ≥ 0.
 */
int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize)
{
	Map map = Map(pMap, nMapWidth, nMapHeight, nTargetX, nTargetY);

	if (nStartX == nTargetX && nStartY == nTargetY) {
		*pOutBuffer = map.Offset(nTargetX, nTargetY);
		return 0;
	}

	int rv = -1;

	NodeQueue queue;
	PNode startNode = std::make_shared<Node>(map, nStartX, nStartY, PNode(), queue);
	size_t mem_use;

	const int target = map.Offset(nTargetX, nTargetY);

	std::string input;
	while(!queue.empty()) {
		PNode curNode(*queue.begin());
		if (curNode->Offset() == target) {
			rv = curNode->FCost();
			PNode n = curNode;
			int i = curNode->GCost() - 1;
			while (n->BestParent() != startNode) {
				pOutBuffer[i--] = n->Offset();
				n = n->BestParent();
			}
			break;
		}

		if (curNode->FCost() < std::numeric_limits<float>::max())
			map.Expand(curNode, queue);
		NodeQueue::iterator cur_it = queue.find(curNode);
		queue.erase(cur_it);

		for (const PNode &nextNode : curNode->Children()) {
			if (nextNode->Offset() != target && nextNode->GCost() >= nOutBufferSize)
				nextNode->SetHCost(std::numeric_limits<float>::max(), queue);
			else if (curNode->FCost() > nextNode->FCost())
				nextNode->SetHCost(curNode->HCost(), queue);
		}

		mem_use = Node::Count() * sizeof(Node) + sizeof(PNode) * queue.size();
		if (mem_use >= _MAX_MEM) {
			const PNode &n = *queue.rbegin();
			n->Erase(queue);
			for (const PNode &p : n->Parents())
				queue.insert(p);
		}

		std::vector<std::string> pic = Paint(map, curNode);
		pic[nTargetY+2][nTargetX+1] = 'X';
		pic[nStartY+2][nStartX+1] = '+';
		for (std::string &l : pic)
			std::cout << l << std::endl;
		std::cout << Node::Count() << " Nodes, " << queue.size() << " Leaves." << std::endl;
		std::cout << "Path length: " << curNode->GCost() << "/" << nOutBufferSize << std::endl;
		std::cout << "Mem: " << float(mem_use)/1000 << " kB."
				<< std::endl << std::endl;

		if (input != "f") {
			std::cout << "Enter \"f\" to finish or press Enter for next step: ";
			std::getline(std::cin, input);
		}
	}
	startNode->Erase(queue);
	return rv;
}


bool contains(std::vector<int> v, int i) {
	for (int j : v)
		if (j == i) return true;
	return false;
}


int main(int argc, char **argv)
{
	unsigned char pMap[] =
	{
			1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,0,0,0,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,
			1,1,0,0,0,0,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 4
			0,0,0,0,0,0,0,0,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,
			0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1, // 10
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1, // 15
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1, // 20
			1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 24
	};//    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
	int width = 39;
	int height = 25;
	std::vector<int> res;
	res.resize(600);

	int start_x = 25;
	int start_y = 19;
	int tgt_x = 2;
	int tgt_y = 0;

	int rv = FindPath(start_x, start_y, tgt_x, tgt_y, pMap, width, height, res.data(), res.size());

	return rv;
}


std::vector<std::string> Paint(Map &map, const PNode &node)
{
	std::vector<std::string> rv;
	std::string line = " ";
	for (int x = 0; x < map.Width(); ++x) {
		line += std::to_string(x % 10);
	}
	rv.push_back(line);
	rv.push_back(" " + std::string(map.Width(), '-'));
	for (int y = 0; y < map.Height(); ++y) {
		std::string line = "|";
		for (int x = 0; x < map.Width(); ++x) {
			if (map.Passable(x, y))
				line += " ";
			else
				line += "#";
		}
		line += "| " + std::to_string(y);
		rv.push_back(line);
	}
	rv.push_back(" " + std::string(map.Width(), '-') + " ");

	PNode n = node;
	while (n) {
		rv[map.Y(n->Offset())+2][map.X(n->Offset())+1] = 'o';
		if (!n->Parents().empty())
			n = n->BestParent();
		else
			break;
	}
	return rv;
}





