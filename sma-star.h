/*
 * sma-star.h
 *
 *  Created on: 24.10.2015
 *      Author: ich
 */

#include <set>
#include <vector>
#include <unordered_set>

#ifndef SMA_STAR_H_
#define SMA_STAR_H_

#define _CART_TO_OFFSET(x, y, width) width * y + x
#define _MAX_MEM 80*1024

class Node;
class Map;
struct CompareByCostEstimate;

typedef std::shared_ptr<Node> PNode;
typedef std::weak_ptr<Node> WeakPNode;
typedef std::set<PNode, CompareByCostEstimate> NodeQueue;
typedef std::vector<PNode> Path;


template<class Ptr_t>
struct HashByOffset {
	size_t operator() (const Ptr_t &node) const;
};


template<class Ptr_t>
struct EqualByOffset : public std::binary_function<Ptr_t, Ptr_t, bool> {
	bool operator() (const Ptr_t &node1, const Ptr_t &node2) const;
};


typedef std::unordered_set<PNode, HashByOffset<PNode>, EqualByOffset<PNode>> NodeSet;

typedef std::unordered_set<WeakPNode, HashByOffset<WeakPNode>, EqualByOffset<WeakPNode>> WeakNodeSet;


class Node {
public:
	Node(const Map &map, const int x, const int y, const PNode &parent, NodeQueue &queue);
	~Node();

	void Erase(NodeQueue &queue, bool eraseFromQ = true);
	void SimplifyPath(const Map &map, NodeQueue &queue);
	std::vector<PNode> FullPath();

	static const unsigned int Count();
	inline const int    Offset() const;
	inline const float  FCost() const;
	inline const int   &GCost() const;
	inline const float &HCost() const;
	inline NodeSet     &Children();
	inline NodeSet     &Parents();
	inline void         SetHCost(const float hCost, NodeQueue &queue);
	inline const int    Serial() const;

	PNode BestParent() const;


private:
	static WeakNodeSet allNodes_;
	static int count_;

	int offset_;
	const int gCost_;
	float hCost_;
	int serial_;
	NodeSet parents_;
	NodeSet children_;
};


struct CompareByCostEstimate : public std::binary_function<PNode, PNode, bool>{
	bool operator() (const PNode &node1, const PNode &node2) const {
		/* If offsets are equal, the two nodes are identical.
		 * In that case, satisfy !comp(a, b) && !comp(b, a) for identity semantics.
		 * cf. http://en.cppreference.com/w/cpp/container/set
		 * Identity is important in expansion queue so A* doesn't search in circles.
		 */
		if (node1->Offset() == node2->Offset())
			return false;

		// Different nodes but same cost: put newer one in front.
		if (node1->FCost() == node2->FCost())
			return node1->Serial() > node2->Serial();

		return node1->FCost() < node2->FCost();
	}
};



class Map {
public:
	Map(const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
			const int targetX, const int targetY);

	inline unsigned int X(const int offset) const;
	inline unsigned int Y(const int offset) const;

	inline int Offset(const int x, const int y) const;
	void Expand(const PNode &n, NodeQueue &queue) const;
	inline bool Passable(const int x, const int y) const;
	inline float HCost(const int from) const;
	inline int Height() const;
	inline int Width() const;

private:
	const unsigned char *pMap_;
	const int nMapWidth_, nMapHeight_;
	const int targetOffset_;

	static const char searchX_[4];
	static const char searchY_[4];
};


std::vector<std::string> Paint(Map &map, const PNode &node);

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize);





#endif /* SMA_STAR_H_ */
