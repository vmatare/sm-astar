#include <set>
#include <vector>
#include <unordered_set>
#include <array>
#include <unordered_map>
#include <memory>
#include <cmath>

#ifndef SMA_STAR_H_
#define SMA_STAR_H_

#define _CART_TO_OFFSET(x, y, width) width * y + x

#ifndef MAX_MEM
#define MAX_MEM 1024*1024*1024
#endif

class Node;
class Map;
struct CompareByCostEstimate;

typedef std::shared_ptr<Node> PNode;
typedef std::weak_ptr<Node> WeakPNode;
typedef std::vector<PNode> Path;


template<class Ptr_t>
struct HashByOffset {
	size_t operator() (const Ptr_t &node) const;
};


template<class Ptr_t>
struct EqualByOffset : public std::binary_function<Ptr_t, Ptr_t, bool> {
	bool operator() (const Ptr_t &node1, const Ptr_t &node2) const;
};


// Thanks to
// http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


typedef struct {
	int x;
	int y;
} point_t;


typedef std::unordered_set<PNode, HashByOffset<PNode>, EqualByOffset<PNode>> NodeSet;
typedef std::unordered_map<int, WeakPNode, HashByOffset<int>, EqualByOffset<int>> WeakNodeSet;
typedef std::set<PNode, CompareByCostEstimate> NodeQueue;


constexpr double pi() { return std::atan(1)*4; }


class Node {
private:
	Node(const Map &map, const int x, const int y, const PNode &parent, WeakNodeSet &allNodes);
public:
	~Node();
	static PNode Spawn(const Map &map, const int x, const int y, const PNode &parent,
			NodeQueue &queue, WeakNodeSet &allNodes);

	void Erase(NodeQueue &queue, bool eraseFromQ = true);
	bool SimplifyPath(const Map &map, NodeQueue &queue);
	std::vector<PNode> FullPath();
	NodeSet AllParents();

	const int    Offset() const;
	const float  FCost() const;
	const int    GCost() const;
	NodeSet     &Children();
	PNode        Parent();
	void         SetGCost(const int gCost, NodeQueue &queue);
	void         SetFCost(const float fCost, NodeQueue &queue);
	bool         Completed();
	void         Reset();
	unsigned long int ID() const;

	void Backup(NodeQueue &queue);

	PNode ExpandNext(const Map &map, NodeQueue &queue);

private:
	static const point_t expandLeft_[4];
	static const point_t expandDown_[4];
	static const point_t expandUp_[4];
	static const point_t expandRight_[4];

	thread_local static unsigned long int count_;

	int offset_;
	int gCost_;
	float fCost_;
	PNode parent_;
	NodeSet children_;
	WeakNodeSet &allNodes_;
	int x_;
	int y_;
	unsigned char expandIdx_;
	unsigned long int id_;
};


struct CompareByCostEstimate : public std::binary_function<PNode, PNode, bool>{
	bool operator() (const PNode &node1, const PNode &node2) const {
		// Different nodes but same cost: put newer one in front.
		if (node1->FCost() == node2->FCost()) {
			if (node1->GCost() == node2->GCost())
				return node1->ID() > node2->ID();
			return node1->GCost() > node2->GCost();
		}

		return node1->FCost() < node2->FCost();
	}
};



class Map {
public:
	Map(const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
			const int targetX, const int targetY);

	int X(const int offset) const;
	int Y(const int offset) const;

	int   Offset(const int x, const int y) const;
	bool  Passable(const int x, const int y) const;
	float HCost(const int from) const;
	float HCost2(const int from) const;
	int   Height() const;
	int   Width() const;
	float Phi(const Node *) const;

	const int nMapWidth_, nMapHeight_;
	const int targetOffset_;
private:
	const unsigned char *pMap_;

	static const char searchX_[4];
	static const char searchY_[4];
};


int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize);




#endif /* SMA_STAR_H_ */
