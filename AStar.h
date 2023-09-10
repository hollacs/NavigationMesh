#pragma once

#include "Navmesh.h"

class PathResult
{
public:
	void Insert(Navmesh* pMesh, NavmeshBridge* pBridge);

	const std::pair<Navmesh*, NavmeshBridge*>& Get(size_t i)
	{
		return m_path[i];
	}

	std::vector<Vector> Smooth(const Vector& sp, const Vector& ep, int maxPts = 0, float bodysize = 18);

	size_t Size() { return m_path.size(); }

private:

	float TriArea(const Vector& a, const Vector& b, const Vector& c) const;
	bool vequal(const Vector& a, const Vector& b) const;

	std::vector<std::pair<Navmesh*, NavmeshBridge*>> m_path;
};

struct AStarNode
{
	AStarNode()
		: pMesh(nullptr), pBridge(nullptr), f(0), g(0), h(0), isOpen(false), isClosed(false), pParent(nullptr)
	{}

	AStarNode(Navmesh* mesh, float f, float g, float h)
		: pMesh(mesh), pBridge(nullptr), f(f), g(g), h(h), isOpen(false), isClosed(false), pParent(nullptr)
	{}

	Navmesh* pMesh;
	NavmeshBridge* pBridge;
	float f, g, h;
	bool isOpen, isClosed;
	AStarNode* pParent;
};

struct ANodeGreater {
	bool operator()(const AStarNode* pa, const AStarNode* pb) const {
		return pa->f > pb->f;
	}
};

class AStar
{
public:
	static bool FindPath(const Vector& sp, const Vector& ep, Navmesh* pStart, Navmesh* pEnd, PathResult &path);

private:
	AStar() {}
};