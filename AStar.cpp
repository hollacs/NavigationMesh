#include "AStar.h"

#include <algorithm>
#include <unordered_map>

void PathResult::Insert(Navmesh* pMesh, NavmeshBridge* pBridge)
{
	m_path.insert(m_path.begin(), std::pair<Navmesh*, NavmeshBridge*>(pMesh, pBridge));
}

std::vector<Vector> PathResult::Smooth(const Vector& sp, const Vector& ep, int maxPts, float bodysize)
{
	std::vector<Vector> newPath;

	Vector portalApex, portalLeft, portalRight;
	int apexIndex = 0, leftIndex = 0, rightIndex = 0;

	portalApex = sp;
	newPath.push_back(portalApex);

	int size = (int)m_path.size();
	int num = min(size, maxPts);
	if (num == 0)
		num = size;

	for (int i = 1; i < num; i++)
	{
		Vector left = m_path[i].second->GetLeftPos();
		Vector right = m_path[i].second->GetRightPos();
		Vector center = m_path[i].second->GetJunction();

		if (TriArea(portalApex, left, right) < 0.0f)
			std::swap(left, right);

		float dist = (left - center).Length();
		dist = max(dist - bodysize, 0);

		left = center + (left - center).Normalize() * dist;
		right = center + (right - center).Normalize() * dist;

		if (portalLeft == Vector())
		{
			portalLeft = left;
			portalRight = right;
			apexIndex = i;
			leftIndex = i;
			rightIndex = i;
		}

		if (TriArea(portalApex, portalRight, right) <= 0.0f)
		{
			if (vequal(portalApex, portalRight) || TriArea(portalApex, portalLeft, right) > 0.0f)
			{
				portalRight = right;
				rightIndex = i;
			}
			else
			{
				portalApex = portalLeft;
				apexIndex = leftIndex;

				portalLeft = portalApex;
				portalRight = portalApex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;

				newPath.push_back(portalApex);

				i = apexIndex;
				continue;
			}
		}

		if (TriArea(portalApex, portalLeft, left) >= 0.0f)
		{
			if (vequal(portalApex, portalLeft) || TriArea(portalApex, portalRight, left) < 0.0f)
			{
				portalLeft = left;
				leftIndex = i;
			}
			else
			{
				portalApex = portalRight;
				apexIndex = rightIndex;

				portalLeft = portalApex;
				portalRight = portalApex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;

				newPath.push_back(portalApex);

				i = apexIndex;
				continue;
			}
		}

		
	}

	if (num == m_path.size())
		newPath.push_back(ep);

	return newPath;
}

float PathResult::TriArea(const Vector& a, const Vector& b, const Vector& c) const
{
	float ax = b[0] - a[0];
	float ay = b[1] - a[1];
	float bx = c[0] - a[0];
	float by = c[1] - a[1];

	return bx * ay - ax * by;
}

bool PathResult::vequal(const Vector& a, const Vector& b) const
{
	static const float eq = 0.001f * 0.001f;
	return (a - b).Length() < eq;
}

bool AStar::FindPath(const Vector& sp, const Vector& ep, Navmesh* pStart, Navmesh* pEnd, PathResult& path)
{
	std::vector<AStarNode*> openList, closedList;
	std::unordered_map<Navmesh*, AStarNode> nodeMap;

	float score = (sp - ep).Length();
	AStarNode node(pStart, score, score, 0);
	node.isOpen = true;

	auto pair = nodeMap.insert(std::pair<Navmesh*, AStarNode>(pStart, node));
	openList.push_back(&pair.first->second);

	// need?
	std::make_heap(openList.begin(), openList.end(), ANodeGreater());

	while (!openList.empty())
	{
		AStarNode* pCurrent = openList.front();

		if (pCurrent->pMesh == pEnd)
		{
			do {
				path.Insert(pCurrent->pMesh, pCurrent->pBridge);
				pCurrent = pCurrent->pParent;

			} while (pCurrent != nullptr);

			return true;
		}

		pCurrent->isOpen = false;
		pCurrent->isClosed = true;

		std::pop_heap(openList.begin(), openList.end(), ANodeGreater());
		openList.pop_back();

		closedList.push_back(pCurrent);

		Navmesh* pCurrentMesh = pCurrent->pMesh;
		auto &bridges = pCurrentMesh->GetBridges();

		for (auto it = bridges.begin(); it != bridges.end(); it++)
		{
			NavmeshBridge* pBridge = it._Ptr;
			Navmesh* pChildMesh = pBridge->GetMesh();
			AStarNode* pChild = nullptr;

			auto iter = nodeMap.find(pChildMesh);
			if (iter == nodeMap.end())
			{
				node = AStarNode(pChildMesh, FLT_MAX, FLT_MAX, FLT_MAX);
				pair = nodeMap.insert(std::pair<Navmesh*, AStarNode>(pChildMesh, node));
				pChild = &pair.first->second;
			}
			else
			{
				pChild = &iter->second;

				if (pChild->isClosed)
					continue;
			}

			score = pCurrent->g + (pCurrentMesh->GetOrigin() - pBridge->GetJunction()).Length() + (pBridge->GetJunction() - pChildMesh->GetOrigin()).Length();
			if (score < pChild->g)
			{
				pChild->pParent = pCurrent;
				pChild->g = score;
				pChild->h = (pChildMesh->GetOrigin() - ep).Length();
				pChild->f = pChild->g + pChild->h;
				pChild->pBridge = it._Ptr;

				if (!pChild->isOpen)
				{
					pChild->isOpen = true;
					openList.push_back(pChild);
					std::push_heap(openList.begin(), openList.end(), ANodeGreater());
				}
			}
		}
	}

	return false;
}
