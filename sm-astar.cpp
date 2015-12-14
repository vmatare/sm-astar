#include "sm-astar.h"

#include <limits>
#include <cstring>
#include <exception>
#include <iostream>
#include <numeric>

#include "output.h"

const char Map::searchX_[] = {1,  0, -1,  0};
const char Map::searchY_[] = {0, -1,  0,  1};
const point_t Node::expandUp_[4]    = { {0,-1}, {-1,0}, {1,0}, {0,1} }; // up-left-right-down
const point_t Node::expandLeft_[4]  = { {-1,0}, {0,1}, {0,-1}, {1,0} }; // left-down-up-right
const point_t Node::expandDown_[4]  = { {0,1}, {1,0}, {-1,0}, {0,-1} }; // down-right-left-up
const point_t Node::expandRight_[4] = { {1,0}, {0,-1}, {0,1}, {-1,0} };  // right-up-down-left

thread_local unsigned long int Node::count_ = 0;


template<class T> size_t HashByOffset<T>::operator() (const T &node) const
{ return node->Offset(); }
///*
template<> size_t HashByOffset<const std::weak_ptr<Node> &>::operator() (const std::weak_ptr<Node> &node) const
{ return node.lock()->Offset(); }
//*/
template<> size_t HashByOffset<int>::operator() (const int &i) const
{ return i; }

template size_t HashByOffset<PNode>::operator() (const PNode &node) const;
//template size_t HashByOffset<WeakPNode>::operator() (const WeakPNode &node) const;


template<class T> bool EqualByOffset<T>::operator() (
		const T &node1,
		const T &node2) const
{ return node1->Offset() == node2->Offset(); }
///*
template<> bool EqualByOffset<std::weak_ptr<Node>>::operator() (
		const std::weak_ptr<Node> &node1,
		const std::weak_ptr<Node> &node2) const
{ return node1.lock()->Offset() == node2.lock()->Offset(); }
//*/
template<> bool EqualByOffset<int>::operator() (const int &lhs, const int &rhs) const
{ return lhs == rhs; }

template bool EqualByOffset<const PNode &>::operator() (const PNode &node1, const PNode &node2) const;
//template bool EqualByOffset<const WeakPNode &>::operator() (const WeakPNode &node1, const WeakPNode &node2) const;



Node::Node(const Map &map, const int x, const int y, const PNode &parent, WeakNodeSet &allNodes) :
		offset_(map.Offset(x, y)), gCost_(parent ? parent->gCost_ + 1 : 0),
		fCost_(float(gCost_) + map.HCost(offset_)),
		parent_(parent),
		allNodes_(allNodes), x_(x), y_(y), expandIdx_(0),
		id_(count_++)
{}


Node::~Node()
{
	allNodes_.erase(this->offset_);
}


PNode Node::Spawn(const Map &map, const int x, const int y, const PNode &parent, NodeQueue &queue, WeakNodeSet &allNodes)
{
	WeakNodeSet::iterator n_it = allNodes.find(map.Offset(x, y));
	PNode rv(nullptr);
	if (n_it == allNodes.end()) {
		rv.reset(new Node(map, x, y, parent, allNodes));
		if (parent) {
			NodeSet::iterator pc_it = parent->Children().find(rv);
			if (pc_it != parent->Children().end()) {
				rv = *pc_it;
			} else {
				parent->children_.insert(rv);
			}
		}
		allNodes.insert(std::make_pair<int, WeakPNode>(rv->Offset(), rv));
	} else {
		rv = PNode(n_it->second);
		if (rv->Parent() != parent) {
			if (rv->GCost() < parent->GCost()+1) {
				rv.reset();
			} else {
				if (parent->GCost()+1 < rv->GCost())
					rv->SetGCost(parent->GCost()+1, queue);
				rv->parent_->children_.erase(rv);
				rv->parent_ = parent;
				parent->children_.insert(rv);
			}
		}
		else
			rv.reset();
	}
	return std::move(rv);
}


NodeSet Node::AllParents()
{
	NodeSet rv;
	NodeSet p_parents = parent_->AllParents();
	if (parent_)
		rv.insert(parent_);
	rv.insert(p_parents.begin(), p_parents.end());
	return std::move(rv);
}


void Node::Erase(NodeQueue &queue, bool eraseFromQ)
{
	PNode sh_this(allNodes_.find(offset_)->second);
	while (!children_.empty()) {
		PNode c = *children_.begin();
		c->Erase(queue);
	}
	if (parent_) {
		parent_->children_.erase(sh_this);
	}
	if (eraseFromQ)
		queue.erase(sh_this);
}


bool Node::SimplifyPath(const Map &map, NodeQueue &queue)
{
	Path path(std::move(FullPath()));

	bool rv = false;
	Path::reverse_iterator path_it = path.rbegin();
	PNode n1, n2, n3;

	n2 = *path_it;
	n3 = *(++path_it);

	while (path_it + 1 != path.rend() && !rv) {
		n1 = n2;
		n2 = n3;
		char x = sgn(map.X(n2->offset_) - map.X(n1->offset_));
		char y = sgn(map.Y(n2->offset_) - map.Y(n1->offset_));

		while(++path_it != path.rend()) {
			n3 = *path_it;
			char next_x = sgn(map.X(n3->offset_) - map.X(n2->offset_));
			char next_y = sgn(map.Y(n3->offset_) - map.Y(n2->offset_));
			if (next_x == x && next_y == y && queue.find(n2) == queue.end()) {
				while (!n1->Children().empty()) {
					if ((*n1->Children().begin()) == n2)
						n1->Children().erase(n1->Children().begin());
					else
						(*n1->Children().begin())->Erase(queue);
				}
				n1->Children().insert(n3);
				n2->Children().erase(n3);
				n3->parent_ = n1;
				rv = true;
			} else {
				break;
			}
			n2 = n3;
		}
	}
	return rv;
}


std::vector<PNode> Node::FullPath()
{
	PNode n(allNodes_.find(offset_)->second);
	std::vector<PNode>rv;
	rv.push_back(n);
	while (n->Parent()) {
		n = n->Parent();
		rv.push_back(n);
	}
	return std::move(rv);
}


inline NodeSet    &Node::Children()     { return children_; }
inline PNode       Node::Parent()       { return parent_; }
inline const int   Node::Offset() const { return offset_; }
inline const float Node::FCost()  const { return fCost_; }
inline const int   Node::GCost()  const { return gCost_; }
inline unsigned long int Node::ID() const { return id_; }


inline void Node::SetGCost(int gCost, NodeQueue &queue)
{
	PNode sh_this(allNodes_.find(offset_)->second);
	bool in_q = queue.erase(sh_this) > 0;

	int change = this->gCost_ - gCost;
	this->gCost_ = gCost;
	fCost_ = fCost_ - change;

	if (in_q)
		queue.insert(sh_this);
}


inline void Node::SetFCost(float fCost, NodeQueue &queue)
{
	PNode sh_this(allNodes_.find(offset_)->second);
	bool in_q = queue.erase(sh_this) > 0;
	this->fCost_ = fCost;
	if (in_q)
		queue.insert(sh_this);
}


void Node::Backup(NodeQueue &queue)
{
	if (expandIdx_ > 3) {
		int best_f = std::numeric_limits<int>::max();
		for (const PNode &c : children_) {
			if (c->FCost() < best_f)
				best_f = c->FCost();
		}
		if (best_f < std::numeric_limits<int>::max() && best_f != FCost() ) {
			SetFCost(best_f, queue);
			if (parent_)
				parent_->Backup(queue);
		}
	}
}


PNode Node::ExpandNext(const Map &map, NodeQueue &queue)
{
	float phi = map.Phi(this);
	const point_t *expandSeq;

	if (phi >= 0.25*pi() && phi < 0.75*pi())
		expandSeq = Node::expandUp_;
	else if (phi >= 0.75*pi() && phi < 1.25*pi())
		expandSeq = Node::expandLeft_;
	else if (phi >= 1.25*pi() && phi < 1.75*pi())
		expandSeq = Node::expandDown_;
	else
		expandSeq = Node::expandRight_;

	PNode rv(nullptr);
	for (; !rv; ++expandIdx_) {
		if (expandIdx_ > 3) {
			return PNode(nullptr);
		}

		int x = map.X(Offset()) + expandSeq[expandIdx_].x;
		int y = map.Y(Offset()) + expandSeq[expandIdx_].y;

		if (!map.Passable(x, y))
			continue;

		if (parent_ && parent_->parent_) {
			unsigned int p_dx = std::abs(x - map.X(parent_->parent_->Offset()));
			unsigned int p_dy = std::abs(y - map.Y(parent_->parent_->Offset()));

			if ((p_dx == 0 && p_dy < 2) || (p_dx < 2 && p_dy == 0)) {
				continue;
			}
		}

		PNode sh_this(allNodes_.find(offset_)->second);

		rv = Spawn(map, x, y, sh_this, queue, allNodes_);
	}
	return std::move(rv);
}


bool Node::Completed()
{ return expandIdx_ > 3; }


void Node::Reset()
{ expandIdx_ = 0; }


Map::Map(const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
		const int targetX, const int targetY) :
		nMapWidth_(nMapWidth),
		nMapHeight_(nMapHeight),
		targetOffset_(_CART_TO_OFFSET(targetX, targetY, nMapWidth)),
		pMap_(pMap)
{}

inline int Map::X(const int offset) const
{ return offset % nMapWidth_; }

inline int Map::Y(const int offset) const
{ return offset / nMapWidth_; }

inline int Map::Offset(const int x, const int y) const
{ return _CART_TO_OFFSET(x, y, nMapWidth_); }

int Map::Width() const
{ return nMapWidth_; }

int Map::Height() const
{ return nMapHeight_; }


inline bool Map::Passable(const int x, const int y) const
{ return x >= 0 && y >= 0 && x < nMapWidth_ && y < nMapHeight_ && *(pMap_ + Offset(x, y)); }


float Map::HCost(const int from) const
{
	float dx = std::abs(((float)X(targetOffset_)) - X(from));
	float dy = std::abs(((float)Y(targetOffset_)) - Y(from));
	return dx + dy;
}


float Map::HCost2(const int from) const
{
	float dx = std::abs(((float)X(targetOffset_)) - X(from));
	float dy = std::abs(((float)Y(targetOffset_)) - Y(from));
	return std::sqrt(dx + dy);
}


/**
 * @return Bearing of target seen from Node n, 0 <= result <= 2 pi
 */
float Map::Phi(const Node *n) const
{
	float dx = ((int)X(targetOffset_)) - X(n->Offset());
	float dy = ((int)Y(targetOffset_)) - Y(n->Offset());
	float phi = std::atan2(dy, dx);
	if (phi < 0) {
		phi = 2 * pi() + phi;
	}
	return phi;
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
	WeakNodeSet allNodes;
	PNode startNode = Node::Spawn(map, nStartX, nStartY, PNode(), queue, allNodes);
	queue.insert(startNode);
	size_t mem_use;

	const int target = map.Offset(nTargetX, nTargetY);

	std::string input;
	while(!queue.empty()) {
		PNode curNode(*queue.begin());
		if (curNode->Offset() == target) {
			rv = curNode->FCost();
			PNode n = curNode;
			int i = curNode->GCost() - 1;
			while (n->Parent() != startNode) {
				pOutBuffer[i--] = n->Offset();
				n = n->Parent();
			}
			break;
		}

		PNode nextNode(nullptr);
		if (curNode->FCost() < std::numeric_limits<float>::max())
			nextNode = curNode->ExpandNext(map, queue);

		if (curNode->Completed()) {
			curNode->Backup(queue);

			NodeSet::iterator c_it = curNode->Children().begin();
			while (c_it != curNode->Children().end() &&
					allNodes.find((*c_it)->Offset()) != allNodes.end())
				++c_it;

			if (c_it == curNode->Children().end())
				queue.erase(curNode);
		}

		mem_use = allNodes.size() * sizeof(Node) + sizeof(PNode) * queue.size();

		if (nextNode) {
			if (nextNode->Offset() != target && nextNode->GCost() >= nOutBufferSize)
				nextNode->SetFCost(std::numeric_limits<unsigned int>::max(), queue);
			else if (curNode->FCost() > nextNode->FCost())
				nextNode->SetFCost(curNode->FCost(), queue);

			queue.insert(nextNode);

			Paint(map, curNode, startNode);
			std::cout << allNodes.size() << " Nodes, " << queue.size() << " Leaves." << std::endl;
			std::cout << "Path length: " << curNode->GCost() << "/" << nOutBufferSize << std::endl;
			std::cout << "Mem: " << float(mem_use)/1000 << " kB."
					<< std::endl << std::endl;

			if (input != "f") {
				std::cout << "Enter \"f\" to finish or press Enter for next step: ";
				std::getline(std::cin, input);
			}
		}

		if (mem_use >= MAX_MEM) {
			if (!curNode->SimplifyPath(map, queue)) {
				NodeQueue::reverse_iterator q_rit = queue.rbegin();
				PNode n = *q_rit;
				n->Erase(queue);
				queue.insert(n->Parent());
			}
		}
	}
	startNode->Erase(queue);
	return rv;
}


